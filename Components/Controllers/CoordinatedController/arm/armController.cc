#include "armController.h"
#include "coordinatedController.h"
#include "../utils.h"
#include "Common/commonMath.h"

#ifndef NO_TRACLABS_ARM
#include "arm/armController_TracLabs.h"
#endif
#ifndef NO_WAM_ARM
#include "arm/armController_WAM.h"
#endif
#ifndef NO_MARS_ARM_IPC_ARM
#include "arm/armController_MarsArmIPC.h"
#endif

#include <sys/time.h> //only used for temporary timing diagnostics - removable
#include <algorithm>

ArmController *
ArmController::factory (armStyle style,
                        int ee_dof,
                        CoordinatedController *cc)
{
  ArmController *arm = NULL;

  switch (style) {
#ifndef NO_TRACLABS_ARM
    case TRACLABS:
      arm = new ArmController_TracLabs(ee_dof, cc);
      fprintf(stderr, "using TRACLabs arm\n"); break;
#endif
#ifndef NO_WAM_ARM
    case WAM_ARM:
      arm = new ArmController_WAM(ee_dof, cc);
      fprintf(stderr, "using WAM arm\n"); break;
#endif
#ifndef NO_MARS_ARM_IPC_ARM
    case MARS_ARM_IPC:
      arm = new ArmController_MarsArmIPC(ee_dof, cc);
      fprintf(stderr, "using MARS arm with IPC\n"); break;
#endif
    default:
      fprintf(stderr, "unknown arm controller style...exiting\n");
      exit(0);
  }

  return arm;
}

//----------------------------------------------------------------------------------
//    Constructor - populate all the necessary parameters
//----------------------------------------------------------------------------------
ArmController::ArmController(const ArmParams &_params,
                             int ee_dof, CoordinatedController *_cc)
  : params(_params),
    DOF(_params.dof),
    EE_DOF(ee_dof),
    cc(_cc),
    velocityCalculator(ARM_VELOCITY_WINDOW_SIZE,
                       std::make_pair(ColVector(DOF), 0.0))
{
  //intialize all parameters
  currentAngles.resize(DOF); currentAngles.setAll(0.0);
  currentVelocities.resize(DOF); currentVelocities.setAll(0.0);
  desiredVelocities.resize(DOF); desiredVelocities.setAll(0.0);
  currentTorques.resize(DOF); currentTorques.setAll(0.0);
  worldToArmHT.resize(4,4);

  armActive = false;
  armMode = CC_ARM_MODE_NA;
  positionCmd.resize(DOF); positionCmd.setAll(0.0);
  velocityCmd.resize(DOF); velocityCmd.setAll(0.0);

  //used in Jacobian and manipulability calculations
  J.resize(EE_DOF,DOF);

  //upper triangular matrix of homogeneous transforms and Point3Ds
  //P - joint origin frame, Z - joint z-axis
  //(used to speed up manipulability calculations)
  HT = new Matrix* [DOF+1];
  HT[0] = new Matrix [(DOF+1)*(DOF+1)];
  p = new ColVector* [DOF+1];
  p[0] = new ColVector [(DOF+1)*(DOF+1)];
  z = new ColVector* [DOF+1];
  z[0] = new ColVector [(DOF+1)*(DOF+1)];

  for (int i = 1; i < DOF+1; ++i){
    HT[i] = HT[i-1] + DOF;
    p[i] = p[i-1] + DOF+1;
    z[i] = z[i-1] + DOF+1;
  }

  for( int i = 0; i < DOF+1; i++){
    for( int j = i; j < DOF+1; j++){
      HT[i][j].resize(4,4);
      p[i][j].resize(4);
      z[i][j].resize(4);
    }
  }

  Z.resize(4); Z[0] = 0.0; Z[1] = 0.0; Z[2] = 1.0; Z[3] = 0.0; //z-axis vector
  O.resize(4); O[0] = 0.0; O[1] = 0.0; O[2] = 0.0; O[3] = 1.0; //origin vector
}

ArmController::~ArmController() {
}

//----------------------------------------------------------------------------------
//    updateArm - This should get called only once per iteration to refresh states
//----------------------------------------------------------------------------------
void ArmController::updateArm(Pose worldToArmHt){
  beatHeart();
  updateJointAngles();
  updateJointVelocities();
  //updateJointTorques();  //should make an updateSpecial for each arm type
  //checkLimits();
  worldToArmHT = worldToArmHt.HT();
  precomputeTransforms();

  manageJoints();
}

//----------------------------------------------------------------------------------
//    commandArm - recieve arm command from higher up
//----------------------------------------------------------------------------------
void ArmController::commandArm(CC_ArmCommand *armCmd){

  assert(*(armCmd->cmd.DOF) == DOF);

  ColVector cmdVector;

  switch(armCmd->cmdType){
    case CC_ARM_ABSOLUTE:
      cout<<"Got an absolute joint angles command\n";
      cmdVector = NDofJointDataToColVector(*(armCmd->cmd.position));
      positionCmd = cmdVector;
      armMode = CC_ARM_MODE_POSITION;
      break;
    case CC_ARM_RELATIVE:
      cout<<"Got a relative joint angles command\n";
      cmdVector = NDofJointDataToColVector(*(armCmd->cmd.position));
      positionCmd = cmdVector + currentAngles;
      armMode = CC_ARM_MODE_POSITION;
      break;
    case CC_ARM_VELOCITY:
      cout<<"Got a joint velocity command\n";
      cmdVector = NDofJointDataToColVector(*(armCmd->cmd.velocity));
      velocityCmd = cmdVector;
      armMode = CC_ARM_MODE_VELOCITY;
      break;
  }
}

void ArmController::resetCmd(){
  positionCmd = currentAngles;
  velocityCmd.setAll(0.0);
}

void ArmController::setMode(ArmMode mode){
  armMode = mode;
}

void ArmController::getControlVels(ColVector &controlVels){
  if(cc->getControlMode() == CC_ARM_ONLY){
    double maxSpeed = 0;
    switch(armMode){
      case CC_ARM_MODE_NA: return;
      case CC_ARM_MODE_POSITION:
	controlVels = positionCmd - currentAngles;

	//Cap max velocity
	for(int i = 0; i < DOF; i++){
	  maxSpeed = max(maxSpeed, ABS(controlVels[i]));
	}
	if(maxSpeed > 1.0){
	  cout<<"Capping control vels\n";
	  for(int i = 0; i < DOF; i++){
	    controlVels[i] = 1.0*controlVels[i]/maxSpeed;
	  }
	}

	break;
      case CC_ARM_MODE_VELOCITY:
                           controlVels = velocityCmd;
                           break;
    }
  }

  if(cc->getControlMode() == CC_DECOUPLED_COORDINATED){
    controlVels = params.initialAngles - currentAngles;
  }
}

//----------------------------------------------------------------------------------
//    updateJointVelocities - compute joint velocities
//----------------------------------------------------------------------------------
void ArmController::updateJointVelocities(){
  ColVector diff = velocityCalculator.tail().first
  - velocityCalculator.head().first;
  double dt = velocityCalculator.tail().second
  - velocityCalculator.head().second;
  currentVelocities = diff / dt;
}

//----------------------------------------------------------------------------------
//    getJacobian_WorldFrame - return the [EE_DOF (usually 6) x DOF] arm Jacobian
//                             incorporates world to the arm transform
//----------------------------------------------------------------------------------
Matrix ArmController::getJacobian_WorldFrame(){

  static Point3D tmpP3D;
  Point3D pp[DOF+1], zz[DOF+1];

  for(int i = 0; i < DOF+1; i++) {
    pp[i] = p[0][i];
    zz[i] = z[0][i];

#ifdef DEBUG
    //print out the origin of each joint frame
    fprintf(stderr, "J p: %f %f %f\n", pp[i].x(), pp[i].y(), pp[i].z());
#endif
  }

  //Populate the Jacobian
  for(int i = 0; i < DOF; i++) {
    tmpP3D = zz[i].cross(pp[DOF] - pp[i]);
    J[0][i] = tmpP3D.x(); J[1][i] = tmpP3D.y(); J[2][i] = tmpP3D.z();
    J[3][i] = zz[i].x();  J[4][i] = zz[i].y();  J[5][i] = zz[i].z();
  }

  return J;
}

//----------------------------------------------------------------------------------
//    getJacobian - return the [EE_DOF (usually 6) x DOF] arm Jacobian
//								- computed in the arm base frame
//----------------------------------------------------------------------------------
Matrix ArmController::getJacobian(){

  static ColVector tmpCV(4);
  static Point3D tmpP3D;
  Point3D pp[DOF+1], zz[DOF+1];

  pp[0] = Point3D(O[0], O[1], O[2]);
  zz[0] = Point3D(Z[0], Z[1], Z[2]);

  for(int i = 1; i < DOF+1; i++) {
    tmpCV = HT[0][i]*O; pp[i] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
    tmpCV = HT[0][i]*Z; zz[i] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);

#ifdef DEBUG
    //print out the origin of each joint frame
    fprintf(stderr, "J p: %f %f %f\n", pp[i].x(), pp[i].y(), pp[i].z());
#endif
  }

  //Populate the Jacobian
  for(int i = 0; i < DOF; i++) {
    tmpP3D = zz[i].cross(pp[DOF] - pp[i]);
    J[0][i] = tmpP3D.x(); J[1][i] = tmpP3D.y(); J[2][i] = tmpP3D.z();
    J[3][i] = zz[i].x();  J[4][i] = zz[i].y();  J[5][i] = zz[i].z();
  }

  return J;
}

Matrix ArmController::getJacobian(ColVector jointAngles, Pose worldToArmHT){

  static ColVector tmpCV(4);
  static Matrix ht;
  Point3D pp[DOF+1], zz[DOF+1], tmpP3D;

  //Compute the origin and z-axis vectors at each joint frame
  ht = worldToArmHT.HT();
  for(int i = 0; i < DOF+1; i++) {
    tmpCV = ht*O; pp[i] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
    tmpCV = ht*Z; zz[i] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
    if(i < DOF) ht = ht*DH_to_HT(params.dh.col(i), jointAngles[i]).HT();
  }

//#define DEBUG
#ifdef DEBUG
  cerr << "DH: " << params.dh << endl;
  cerr << "Joint Angles: " << jointAngles << endl;
  cerr << "worldToArm: " << worldToArmHT << endl;

  for(int i = 0; i < DOF+1; i++) {
    //print out the origin of each joint frame
    fprintf(stderr, "J p: %f %f %f\n", pp[i].x(), pp[i].y(), pp[i].z());
  }
  for(int i = 0; i < DOF+1; i++) {
    //print out the origin of each joint frame
    fprintf(stderr, "J z: %f %f %f\n", zz[i].x(), zz[i].y(), zz[i].z());
  }
#endif

  //Populate the Jacobian
  for(int i = 0; i < DOF; i++) {
    tmpP3D = zz[i].cross(pp[DOF] - pp[i]);
    J[0][i] = tmpP3D.x();  J[1][i] = tmpP3D.y();  J[2][i] = tmpP3D.z();
    J[3][i] = zz[i].x();   J[4][i] = zz[i].y();		J[5][i] = zz[i].z();
  }

  return J;
}

ColVector ArmController::getManipulabilityGradient(){

  static ColVector tmpCV(4);
  static Matrix tmpHt, Jt, JT;
  Point3D pp[DOF+1], zz[DOF+1], tmpP3D;
  ColVector gradient(DOF);

  //get current point for numeric differentiation
  getJacobian_WorldFrame(); //populates member variable - J and computes HTs
  Jt = J.t(); JT = J*Jt;
  double val = sqrt(JT.ABSdeterminant());
  //fprintf(stderr, "manip: %f\n", val);

  for(int i = 1; i < DOF+1; i++){
    for(int j = 0; j < DOF+1; j++) {
      if(j < i){
        pp[j] = Point3D(p[0][j][0], p[0][j][1], p[0][j][2]);
        zz[j] = Point3D(z[0][j][0], z[0][j][1], z[0][j][2]);
      }
      if(j == i){
        tmpHt = HT[0][i-1]*DH_to_HT(params.dh.col(i-1),
                                    currentAngles[i-1]+0.01).HT();
        tmpCV = tmpHt*O; pp[j] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
        tmpCV = tmpHt*Z; zz[j] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
      }
      if(j > i){
        //tmpHt should be set above
        tmpCV = tmpHt*p[i+1][j]; pp[j] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
        tmpCV = tmpHt*z[i+1][j]; zz[j] = Point3D(tmpCV[0], tmpCV[1], tmpCV[2]);
      }
    }

#ifdef DEBUG
    for(int j = 0; j < DOF+1; j++) {
      //print out the origin of each joint frame
      fprintf(stderr, "G p: %f %f %f\n", pp[j].x(), pp[j].y(), pp[j].z());
    }
#endif

    //Populate the Jacobian
    for(int j = 0; j < DOF; j++) {
      tmpP3D = zz[j].cross(pp[DOF] - pp[j]);
      J[0][j] = tmpP3D.x(); J[1][j] = tmpP3D.y(); J[2][j] = tmpP3D.z();
      J[3][j] = zz[j].x();  J[4][j] = zz[j].y();  J[5][j] = zz[j].z();
    }

    Jt = J.t(); JT = J*Jt;
    gradient[i-1] = (sqrt(JT.ABSdeterminant()) - val)/0.01;
  }

  return gradient*10;
}

void ArmController::precomputeTransforms(){

  //precompute transform information used in Jacobian and manipulability calculations
  //populate diagonal first
  HT[0][0] = worldToArmHT;
  for( int i = 1; i < DOF+1; i++)
    HT[i][i] = DH_to_HT( params.dh.col(i-1), currentAngles[i-1] ).HT();

  //populate upper triangle
  for( int i = 0; i < DOF+1; i++){
    for( int j = i+1; j < DOF+1; j++)
      HT[i][j] = HT[i][j-1] * HT[j][j];
  }

  // the homogeneous matrix combinations generated...
  //	0 01 012 0123 01234 012345 0123456 01234567
  //		 1	12	123	 1234  12345  123456  1234567
  //		 		 2   23   234   2345   23456   234567
  //		 		      3    34    345    3456    34567
  //		 		            4     45     456     4567
  //		 		                   5      56      567
  //		 		                           6       67
  //		 		                                    7

  //Compute the origin and z-axis vectors at each joint frame
  for(int i = 0; i < DOF+1; i++) {
    for(int j = i; j < DOF+1; j++) {
      p[i][j] = HT[i][j]*O;
      z[i][j] = HT[i][j]*Z;
    }
  }
}

//----------------------------------------------------------------------------------
//    getSkeleton -
//----------------------------------------------------------------------------------
vector<Geom3> ArmController::getSkeleton(){

  static ColVector start(3);
  static ColVector direction(3);
  static FrReal magnitude;
  vector<Geom3> skeleton;

  int skeletonLength = 1;
  //int link[4]; link[0] = 0; link[1] = 2; link[2] = 4; link[3] = 6;
  int link[1]; link[0] = 6;
  //build the skeleton for obstacle avoidance
  for(int i = 0; i < skeletonLength; i++){
    for(int j = 0; j < 3; j++){
      start[j] = p[0][link[i]][j];
      direction[j] = p[0][link[i]+1][j] - start[j];
    }
    magnitude = direction.Normalize()/2.0;
    start = start + direction*magnitude; //origin should be in center of segment
    skeleton.push_back(Segment3(start, direction, magnitude));
  }

  return skeleton;
}

//----------------------------------------------------------------------------------
//    getSkeleton -
//----------------------------------------------------------------------------------
vector<Geom3> ArmController::getSkeleton(int axis, double Vdt){
  static Matrix tmpHt;
  ColVector *pp = new ColVector [DOF+1];
  for(int j = 0; j < DOF+1; j++) pp[j].resize(3);
  vector<Geom3> skeleton;

  axis++;
  for(int j = 0; j < DOF+1; j++) {
    if(j < axis){
      pp[j] = p[0][j];
    }
    if(j == axis){
      tmpHt = HT[0][axis-1]*DH_to_HT(params.dh.col(axis-1),
                                     currentAngles[axis-1]+Vdt).HT();
      pp[j] = tmpHt*O;
    }
    if(j > axis){
      pp[j] = tmpHt*p[axis+1][j]; //tmpHt should be set above
    }
  }


  static ColVector start(3);
  static ColVector direction(3);
  static FrReal magnitude;

  int skeletonLength = 1;
  //int link[4]; link[0] = 0; link[1] = 2; link[2] = 4; link[3] = 6;
  int link[1]; link[0] = 6;
  //build the skeleton for obstacle avoidance
  for(int i = 0; i < skeletonLength; i++){
    for(int j = 0; j < 3; j++){
      start[j] = pp[link[i]][j];
      direction[j] = pp[link[i]+1][j] - start[j];
    }
    magnitude = direction.Normalize()/2.0;
    start = start + direction*magnitude; //origin should be in center of segment
    skeleton.push_back(Segment3(start, direction, magnitude));
  }

  return skeleton;
  delete pp;
}

//----------------------------------------------------------------------------------
//    getForwardKinematics_WorldFrame - return Pose of the end effector
//                                      includes the world to arm transform
//----------------------------------------------------------------------------------
Pose ArmController::getForwardKinematics_WorldFrame(){
  return Pose(HT[0][DOF]);
}

//----------------------------------------------------------------------------------
//    getForwardKinematics - return Pose of the end effector
//                         - does not include the world to arm transform
//----------------------------------------------------------------------------------
Pose ArmController::getForwardKinematics(){
  return Pose(HT[1][DOF]);
}

//----------------------------------------------------------------------------------
//    getForwardKinematics - return location of the end effector in the arm base frame
//                         - but first apply the given joint velocities for dt seconds
//                         - this function is useful for prediction
//----------------------------------------------------------------------------------
Pose ArmController::getForwardKinematics(ColVector velocities, double dt){
  Pose ht;
  for(int i = 0; i < DOF; i++)
    ht = ht*DH_to_HT( params.dh.col(i), currentAngles[i]+velocities[i]*dt );
  return ht;
}

//----------------------------------------------------------------------------------
//    checkLimits - look for joints that exceed the upper or lower joint limits
//----------------------------------------------------------------------------------
//bool ArmController::checkLimits(){
void ArmController::manageJoints(){
  double diff, weight;
  ColVector manipGrad = getManipulabilityGradient();
  for(int i = 0; i < DOF; i++){
    weight = 1.0;

    //min(upper limits - cur angles, cur angles - lower limits)
    diff = min(params.limits[0][i] - currentAngles[i],
               currentAngles[i] - params.limits[1][i]);
    if(diff < PI/8) weight = 0.2;
    if(fabs(manipGrad[i]) > 0.15) weight = 0.2;
    if(diff < 0) weight = 0.01;
    if(fabs(manipGrad[i]) > 0.30) weight = 0.01;


    //cc->setWeightMatrixDim(cc->BASE_DOF+i, weight);
  }

  //cout << "ManipGrad: " << getManipulabilityGradient() << endl;
}

//----------------------------------------------------------------------------------
//    pauseArm/resumeArm - sets the boolean for arm control
//----------------------------------------------------------------------------------
void ArmController::pauseArm(){
  stop();
  armActive = ARM_DISABLED;
}

void ArmController::resumeArm(){
  armActive = ARM_ENABLED;
  stop();
}


void ArmController::goToInitialAngles(){
  positionCmd = params.initialAngles;
  armMode = CC_ARM_MODE_POSITION;
}

//----------------------------------------------------------------------------------
//    DH_to_HT - converts a vector of the 4 DH parameters and the current joint
//						 - angle q to a homogenous transform
//----------------------------------------------------------------------------------
Pose ArmController::DH_to_HT(ColVector DH, double q){
  Pose A(0,0,DH[1],0,0,DH[0]+q);
  Pose B(DH[2],0,0,DH[3],0,0);
  return A*B;
}
