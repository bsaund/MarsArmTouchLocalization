/*****************************************************************
 *
 * Test program for tactile localization using the MARS arm and 
 * JR3 force sensor
 *
 *****************************************************************/

#include <unistd.h>
#include <ipc/ipc.h>
#include "Components/Controllers/CoordinatedController/coordinatedController-ipc.h"
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include "Common/Constraint.xdr.h"
#include "Common/ConstrainedMove.xdr.h"
#include "Common/ConstrainedMoveMessages.h"
#include "particleFilter.h"
#include "tactileLocalizationUtils.h"


static TLU::Status status;


static void forceSensorNoiseHnd (MSG_INSTANCE msg, void *callData,
				 void* clientData)
{
  POSE_6DOF_TYPE forceSensorNoise = *((POSE_6DOF_TYPE*) callData);
  status.forceNoise = Pose(forceSensorNoise);
  status.calibrated = true;
  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}

static void statusHnd(MSG_INSTANCE msg, void *callData, void* clientData)
{
  CoordinatedControllerStatus* ccStatus = (CoordinatedControllerStatus *)callData;

  if (ccStatus->eeStatus.cur.poseLen > 0)
    status.eePose = ccStatus->eeStatus.cur.pose[0];

  if (ccStatus->eeStatus.goal.poseLen > 0)
    status.eeGoalPose = ccStatus->eeStatus.goal.pose[0];

  if (ccStatus->manipStatus.cur.positionLen > 0)
    status.jointAngles = ccStatus->manipStatus.cur.position[0];

  if (ccStatus->manipStatus.cur.velocityLen > 0)
    status.jointVels = ccStatus->manipStatus.cur.velocity[0];

  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}

static void constraintViolatedHnd (MSG_INSTANCE msg, void *callData,
				   void* clientData)
{
  ConstraintsType *violated = (ConstraintsType *)callData;
  //cerr << "constraintViolatedHnd" << endl;
  status.touched = true;
  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}

static void constrainedMoveHnd (MSG_INSTANCE msg, void *callData,
				void* clientData)
{
  ConstrainedMoveStatus *moveStatus = (ConstrainedMoveStatus *)callData;
  status.moveDone = true;
  status.eeEndPose = moveStatus->endPose;
  cerr << "constrainedMoveHnd: " << status.moveDone << " " << status.eeEndPose;
  if (!moveStatus->success) {
    if (moveStatus->violatedStopConstraints.numLists > 1) {
      cerr << "Need to handle multiple violations" << endl; exit(-1);
    } else {
      cerr << "   viol: " << moveStatus->violatedStopConstraints.list[0].list[0].control << endl;
      if (moveStatus->violatedStopConstraints.list[0].list[0].control ==
	  ForceConstraint) {
	status.touched = true;
      }
    }
  }
  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}

static void ipcInit ()
{
  IPC_connect("tactileLocalize");

  IPC_subscribeData(FORCE_SENSOR_NOISE_MSG, forceSensorNoiseHnd, NULL);
  IPC_subscribeData(CC_STATUS_MSG, statusHnd, NULL);
  //IPC_subscribeData(CONSTRAINT_VIOLATED_MSG, constraintViolatedHnd, NULL);
  IPC_subscribeData(CONSTRAINED_MOVE_DONE, constrainedMoveHnd, NULL);
}

static bool moveToPose (Pose pose)
{
  // Activate arm
  int active = 1;
  IPC_publishData(CC_TOGGLE_EE_ACTIVE_MSG, &active);
  IPC_publishData(CC_TOGGLE_ARM_ACTIVE_MSG, &active);

  POSE_6DOF_TYPE pose6DOF(pose);
  EE_DATA_TYPE eeData(&pose6DOF, NULL);
  CC_EECommand eeCmd(CC_EE_ABSOLUTE, eeData);
  IPC_publishData(CC_EE_CMD, &eeCmd);

  // Wait to get to desired position
  do {
    IPC_listenWait(100);
  } while (TLU::PoseDiff(pose, status.eePose) > 0.01);
}

static bool moveToRelativePose (Pose relPose)
{
  // Activate arm
  int active = 1;
  IPC_publishData(CC_TOGGLE_EE_ACTIVE_MSG, &active);
  IPC_publishData(CC_TOGGLE_ARM_ACTIVE_MSG, &active);

  POSE_6DOF_TYPE relMove(relPose);
  EE_DATA_TYPE eeData3(&relMove, NULL);
  CC_EECommand eeCmd3(CC_EE_RELATIVE, eeData3);
  IPC_publishData(CC_EE_CMD, &eeCmd3);

  // Should check for move getting to desired position,
  // but didn't feel like figuring out the end pose
  sleep(1);
}

static bool guardedMoveToPose (Pose pose)
{
  // Activate arm
  int active = 1;
  IPC_publishData(CC_TOGGLE_EE_ACTIVE_MSG, &active);
  IPC_publishData(CC_TOGGLE_ARM_ACTIVE_MSG, &active);

  // Set up force constraint, in the Z direction
  Constraint forceConstraint(ForceConstraint, LocalFrameConstraint,(char *)"Z",
			     MAX(status.forceNoise.z()*1.5, 1.5),
			     LessConstraint, true);
  //cerr << forceConstraint << endl;
  DNConstraintList constraintList(forceConstraint);
  Constraint poseConstraint(PoseConstraint, SpecifiedFrameConstraint,
			    (char *)"XYZ", 0.001, GreaterConstraint,
			    true, 0.0, pose);
  //cerr << poseConstraint << endl;
  constraintList.addConstraint(poseConstraint);
  IPC_publishData(SET_HARD_CONSTRAINTS_MSG, &constraintList);

  // Move to startPose
  cerr << "Guarded move to pose: " << pose << endl;
  moveToPose(pose);

  status.moveDone = false;
  do {
    IPC_listenWait(100);
  } while (!status.moveDone);
  ConstraintsType hard = HARD_CONSTRAINTS;
  IPC_publishData(REMOVE_CONSTRAINTS_MSG, &hard);
}

static bool touchPoint (Pose startPose, double forwardMove, bool calibrate)
{
  // Move to startPose
  cerr << "Move to start pose: " << startPose << endl;
  moveToPose(startPose);

  // Calibrate the force sensor
  if (calibrate) {
    sleep(1); // Let things settle down first
    int tmp = 1; // Dummy
    status.calibrated = false;
    IPC_publishData(FORCE_RECALIBRATION_MSG, &tmp);
    do {
      IPC_listenWait(100);
    } while (!status.calibrated);
    cerr << "Force noise: " << status.forceNoise << endl;
  }

  // Move in guarded-move velocity mode,
  //  but stop if travelled too far without touching
  Pose eeRelPose(0, 0, forwardMove, 0, 0, 0);
  Pose goalPose = status.eePose * eeRelPose;
  cerr << status.eePose << endl << eeRelPose << endl << goalPose << endl;

  // Set up force constraint, in the Z direction
  Constraint forceConstraint(ForceConstraint, LocalFrameConstraint,(char *)"Z",
			     MAX(status.forceNoise.z()*1.5, 1.5),
			     LessConstraint, true);
  DNConstraintList constraintList(forceConstraint);
  //cerr << forceConstraint << endl;
  // Set up pose constraint, further than the estimated goal pose
  Constraint poseConstraint(PoseConstraint, SpecifiedFrameConstraint,
			    (char *)"XYZ", forwardMove, LessConstraint,
			    true, 0.0, status.eePose);
  constraintList.addConstraint(poseConstraint);
  //cerr << poseConstraint << endl;
  IPC_publishData(SET_HARD_CONSTRAINTS_MSG, &constraintList);

  cerr << "Guarded move: " << endl;
  POSE_6DOF_TYPE eeVels(0, 0, 0.01, 0, 0, 0);
  EE_DATA_TYPE eeData(NULL, &eeVels);
  CC_EECommand eeCmd(CC_EE_VELOCITY, eeData);
  IPC_publishData(CC_EE_CMD, &eeCmd);

  status.touched = status.moveDone = false;
  do {
    IPC_listenWait(100);
  } while (!status.moveDone);

  if (status.touched) {
    cerr << "Touched at: " << status.eeEndPose << endl;
  } else {
    cerr << "Stopped without touching: " << status.eeEndPose << endl;
  }
  ConstraintsType hard = HARD_CONSTRAINTS;
  IPC_publishData(REMOVE_CONSTRAINTS_MSG, &hard);

  // Return to start position
  if (status.touched) { // If touched, back straight out a bit, first
    cerr << "Back up: " << endl;
    moveToRelativePose(Pose(0, 0, -0.01, 0, 0, 0));
  }

  cerr << "Return to start pose from: " << status.eePose << endl;
  moveToPose(startPose);
}

static void updateTouchStats (const Pose &pose, int num,
			      ColVector &mean, ColVector &std)
{
  ColVector posevec(6), sum(6), sumSq(6);
  posevec[0] = pose.x();  posevec[1] = pose.y();  posevec[2] = pose.z();
  posevec[3] = pose.rx(); posevec[4] = pose.ry(); posevec[5] = pose.rz();
  if (num > 1) {
    sum = mean * (num-1);
    for (int i=0; i<6; i++) sumSq[i] = (SQ(mean[i]) + SQ(std[i])) * (num-1);
  } else sum = sumSq = 0.0;
  sum += posevec;
  for (int i=0; i<6; i++) sumSq[i] += SQ(posevec[i]);

  mean = sum/num;
  if (num > 1) 
    for (int i=0; i<6; i++) std[i] = sqrt(sumSq[i]/num - SQ(mean[i]));
}

static void addObservation (particleFilter &pfilter, Pose pose, bool touched)
{
  if (touched) {
    double obs[3] = {pose.x(), pose.y(), pose.z()};
    cout << "obs: " << obs[0] << ' ' << obs[1] << ' ' << obs[2] << endl;
    pfilter.addObservation(obs);
    particleFilter::cspace X_est[2];
    pfilter.estimatedDistribution(X_est);
    cout << "est: " << X_est[0][0] << ' ' << X_est[0][1] << ' ' << X_est[0][2]
	 << " (" << X_est[1][0] << ")" << endl;
  } else {
    cerr << "NEED TO DO SOMETHING WHEN TARGET IS MISSED\n";
  }
}

static void computeInitialDistribution (particleFilter::cspace binit[2],
					Pose touch1, Pose touch2, Pose touch3)
{
  // Compute the plane parameters for these three points
  ColVector p1(3), p2(3), p3(3), d1, d2, cross;
  p1[0] = touch1.x(); p1[1] = touch1.y(); p1[2] = touch1.z();
  p2[0] = touch2.x(); p2[1] = touch2.y(); p2[2] = touch2.z();
  p3[0] = touch3.x(); p3[1] = touch3.y(); p3[2] = touch3.z();

  //cerr << p1 << p2 << p3;

  d1 = p2-p1; d2 = p2-p3;
  //cerr << d1 << d2;
  cross = d1.Cross(d2);
  //cerr << cross;

  double D = cross[0]*p1[0] + cross[1]*p1[1] + cross[2]*p1[2];
  binit[0][0] = cross[1]/cross[0];
  binit[0][1] = cross[2]/cross[0];
  binit[0][2] = -D/cross[0];
  cerr << "Init: " << binit[0][0] << " " << binit[0][1] << " " << binit[0][2] << endl;
  binit[1][0] = binit[1][1] = binit[1][2] = 0.1;
}

static Pose poseAt (double y, double z, particleFilter::cspace plane)
{
  double t = atan(plane[0]);
  double p = acos(plane[1]/sqrt(1 + SQ(plane[0]) + SQ(plane[1])));

  //cerr << "t: " << Rad_to_Deg(t) << " p: " << Rad_to_Deg(p) << endl;

  Pose pose(-(plane[0]*y + plane[1]*z + plane[2]), y, z, -t, p, 0);
  cerr << "poseAt: " << pose << endl;
  return pose;
}

static Pose poseNear (double y, double z, double offset,
		      particleFilter::cspace plane)
{
  ColVector normal(3);
  normal[0] = 1; normal[1] = plane[0]; normal[2] = plane[1];
  normal.Normalize();

  Pose point0 = poseAt(y, z, plane);
  return Pose(point0.x()+normal[0]*offset, 
	      point0.y()+normal[1]*offset,
	      point0.z()+normal[2]*offset,
	      point0.rx(), 
	      point0.ry(), 
	      point0.rz());
}

static void moveNearPlane (const particleFilter &pfilter, double dist)
{
  double dtouch = 0.05;
  Pose rotate(0, 0, 0, 0, 0, M_PI/4);
  Pose touchPlane(0, dtouch/2, dtouch/2, 0, 0, 0);
  Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  Pose touch = rotate*(touchBase+touchPlane);
  particleFilter::cspace binit[2];
  pfilter.estimatedDistribution(binit);

  Pose nearPt = poseNear(touch.y(), touch.z(), -dist, binit[0]);
  cerr << "Near pose: " << nearPt << endl;
#ifndef TEST_PF
  guardedMoveToPose(nearPt);
#endif
}

static void userInput (const char *msg)
{
  char buff[80];
  cerr << msg << ": ";
  fgets(buff, 80, stdin);
}



int main ()
{ 
  ipcInit();

#if 0
  Pose touch1(0.404, 0.303, 1.267, 0.0, M_PI/2, M_PI/4);
  ColVector mean(6), std(6);
  int num = 10;
  for (int i=1; i<=num; i++) {
    touchPoint(touch1, 0.1, (i==1)); // Calibrate force sensor just first time
    updateTouchStats(status.eeEndPose, i, mean, std);
    cerr << "Mean: " << mean << "Std: " << std;
  }
#else
  Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  double dtouch = 0.05;
  Pose touchUp(0, 0, dtouch, 0, 0, 0);
  Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  Pose touchSide(0, dtouch, 0, 0, 0, 0);
  Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  bool touched1, touched2, touched3;

  particleFilter pfilter(1000);

  Pose touch1 = rotate*touchBase;
//#define TEST_PF
#ifndef TEST_PF
  touchPoint(touch1, 0.1, true);
  touch1 = status.eeEndPose;
  touched1 = status.touched;
#else
  touched1 = true;
#endif

  Pose touch2 = rotate*(touchBase+touchUp);
#ifndef TEST_PF
  touchPoint(touch2, 0.1, false);
  touch2 = status.eeEndPose;
  touched2 = status.touched;
#else
  touched2 = true;
#endif

  Pose touch3 = rotate*(touchBase+touchUp+touchSide);
#ifndef TEST_PF
  touchPoint(touch3, 0.1, false);
  touch3 = status.eeEndPose;
  touched3 = status.touched;
#else
  touched3 = true;
#endif

  particleFilter::cspace binit[2];

  computeInitialDistribution(binit, touch1, touch2, touch3);
  pfilter.setDistribution(binit);

#if 0
  moveNearPlane(pfilter, 0.01);
  userInput("Initial move");
#endif
  addObservation(pfilter, touch1, touched1);
  addObservation(pfilter, touch2, touched2);
  addObservation(pfilter, touch3, touched3);

  moveToPose(rotate*(touchBase+touchUp+touchSide));
#if 0
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 3 observations");
#endif
  Pose touch4 = rotate*(touchBase+touchSide);
#ifndef TEST_PF
  touchPoint(touch4, 0.1, false);
#else
  status.eeEndPose = touch4; status.touched = true;
#endif
  addObservation(pfilter, status.eeEndPose, status.touched);
#if 0
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 4 observations");
#endif
  Pose touch5 = rotate*(touchBase+touchMiddle);
#ifndef TEST_PF  
  touchPoint(touch5, 0.1, false);
#else
  status.eeEndPose = touch5; status.touched = true;
#endif
  addObservation(pfilter, status.eeEndPose, status.touched);
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 5 observations");
#if 1
  Pose touch6 = rotate*(touchBase+touchDown);
  touchPoint(touch6, 0.1, false);
  addObservation(pfilter, status.eeEndPose, status.touched);
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 6 observations");

  Pose touch7 = rotate*(touchBase+touchDown+touchSide);
  touchPoint(touch7, 0.1, false);
  addObservation(pfilter, status.eeEndPose, status.touched);
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 7 observations");
#endif
#endif
}
