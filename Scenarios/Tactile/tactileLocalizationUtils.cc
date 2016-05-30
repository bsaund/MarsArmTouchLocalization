
#include <unistd.h>
#include <ipc/ipc.h>
#include <cmath>
#include "Components/Controllers/CoordinatedController/coordinatedController-ipc.h"
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include "Common/Constraint.xdr.h"
#include "Common/ConstrainedMove.xdr.h"
#include "Common/ConstrainedMoveMessages.h"
#include "tactileLocalizationUtils.h"
#include "tactileLocalizeMsg.h"

// #include <sys/types.h>
// #include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

static TLU::Status status;
static const char *portName = "/dev/ttyACM1";
static int fd = -1;

//---------------------------------------------
//---Message Handlers from controller----------
//---------------------------------------------

static void forceSensorNoiseHnd (MSG_INSTANCE msg, void *callData,
				 void* clientData)
{
  POSE_6DOF_TYPE forceSensorNoise = *((POSE_6DOF_TYPE*) callData);
  status.forceNoise = Pose(forceSensorNoise);
  status.calibrated = true;
  
  std::cout << "Force noise Handled" << std::endl;
  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}



static void statusHnd(MSG_INSTANCE msg, void *callData, void* clientData)
{
  CoordinatedControllerStatus* ccStatus = (CoordinatedControllerStatus *)callData;

  // std::cout <<"Status Handled" << std::endl;

  if (ccStatus->eeStatus.cur.poseLen > 0)
    status.eePose = ccStatus->eeStatus.cur.pose[0];

  if (ccStatus->eeStatus.goal.poseLen > 0)
    status.eeGoalPose = ccStatus->eeStatus.goal.pose[0];

  if (ccStatus->manipStatus.cur.positionLen > 0){
    status.jointAngles = ccStatus->manipStatus.cur.position[0];
    // cout << status.jointAngles.data[0] << endl;
  }

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

static void configure_port (int fd)
{
  struct termios settings;

  cfsetispeed(&settings, B57600);
  cfsetospeed(&settings, B57600);
  settings.c_cflag &= ~PARENB; // no parity
  settings.c_cflag &= ~CSTOPB; // one stop bit
  settings.c_cflag &= ~CSIZE;  // 8 bit size
  settings.c_cflag |= CS8;

  tcsetattr(fd, TCSANOW, &settings);
}

static int open_port (const char *port)
{
  int fd = open(port, O_RDONLY | O_NOCTTY | O_NDELAY);

  if (fd == -1) {
    cout << "Unable to open " << port << endl;
  } else {
    fcntl(fd, F_SETFL, 0);
    cout << "Port " << port << " is open" << endl;
    configure_port(fd);
  }
  return fd;
}



//---------------------------------------------
//---Init Function, call at beginning----------
//---------------------------------------------



void TLU::ipcInit ()
{
  IPC_connect("tactileLocalize");
  IPC_setCapacity(5);
  IPC_defineMsg(TOUCH_OBSERVATION_MSG, IPC_VARIABLE_LENGTH, TOUCH_OBSERVATION_FORM);
  IPC_defineMsg(TOUCH_LOCATION_MSG, IPC_VARIABLE_LENGTH, TOUCH_LOCATION_FORM);
  IPC_defineMsg(PROCESS_FINISHED_MSG, IPC_VARIABLE_LENGTH, PROCESS_FINISHED_FORM);
  IPC_subscribeData(FORCE_SENSOR_NOISE_MSG, forceSensorNoiseHnd, NULL);
  IPC_subscribeData(CC_STATUS_MSG, statusHnd, NULL);
  //IPC_subscribeData(CONSTRAINT_VIOLATED_MSG, constraintViolatedHnd, NULL);
  IPC_subscribeData(CONSTRAINED_MOVE_DONE, constrainedMoveHnd, NULL);


  // IPC_defineMsg (NDOF_JOINT_ABS_POS_COMMAND_MSG, IPC_VARIABLE_LENGTH, NDOF_JOINT_FMT);
  usleep(10000); // For some reason, need to delay first
  tcflush(fd, TCIOFLUSH);

  fd = open_port(portName);

}

//---------------------------------------------
//----Useful Functions-------------------------
//---------------------------------------------

double TLU::readDistanceProbe(){
  tcflush(fd, TCIOFLUSH);
  sleep(2);
  tcflush(fd, TCIOFLUSH);

  char buffer[80];
  bzero(buffer, sizeof(buffer));
  for (int i=0; i<80;) {
    int retval = ::read(fd, &buffer[i], 1);
    if (retval == -1)
      perror("read");
    else if (retval == 1) {

      // Serial output ends with ^M/CR
      if (buffer[i] == '\n') {
	//std::cout << "backslash n" << std::endl;
	break;
      }
      if (buffer[i] != '\n') i++;
    }
  }

  // std::cout <<buffer <<std::endl;
  double ambiant;
  double distance;

  sscanf(buffer, "Range: %lf \n", &distance);

  std::cout << "Distance is: " << distance << std::endl;

  if(distance == 0){
    std::cout << "Buffer: " << buffer << std::endl;
  }
    
  return distance;
}


bool TLU::moveToAngles(double jointAngles[7])
{
  // Activate arm
  // Deactivate CC
  int inactive = 0;
  int active = 1;
  IPC_publishData(CC_TOGGLE_EE_ACTIVE_MSG, &inactive);
  IPC_publishData(CC_TOGGLE_ARM_ACTIVE_MSG, &active);
  


  // POSE_6DOF_TYPE pose6DOF(pose);
  // EE_DATA_TYPE eeData(&pose6DOF, NULL);
  // CC_EECommand eeCmd(CC_EE_ABSOLUTE, eeData);
  // // IPC_publishData(CC_EE_CMD, &eeCmd);
  // // IPC_publishData(NDOF_JOINT_ABS_POS_COMMAND_MSG, &jointAngles);
  NDofJointData jointAnglesData(7);
  for(int i=0; i<7; i++){
    jointAnglesData.data[i] = jointAngles[i];
  }

  ARM_DATA_TYPE armData(&jointAnglesData, NULL);
  CC_ArmCommand armCmd(CC_ARM_ABSOLUTE, armData);
  IPC_publishData(CC_ARM_CMD, &armCmd);

  // Wait to get to desired position
  do {
    IPC_listenWait(100);
    // std::cout << "Angle diff: " << TLU::angleDiff(jointAnglesData, status.jointAngles) << std::endl
    ;
  } while (TLU::angleDiff(jointAnglesData, status.jointAngles) > 0.04);

}

bool TLU::moveToPose (Pose pose)
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
    // std::cout << "Moving to pose " << std::endl;
  } while (TLU::PoseDiff(pose, status.eePose) > 0.01);
}

bool TLU::moveToRelativePose (Pose relPose)
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


bool TLU::guardedMoveToPose (Pose pose)
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


TLU::TouchStatus TLU::touchPoint (Pose startPose, double forwardMove, bool calibrate)
{
  return TLU::touchPoint(startPose, forwardMove, calibrate, 0.01, false);
}

/* 
 * Moves from startPose forward (in the End Effector Z direction)
 * by forwardMove, stopping if the sensor touches something
 * Optionally calibrate the force sensor
 * Returns a struct of the location of the touch point, and a boolean if contact was made
 */
TLU::TouchStatus TLU::touchPoint (Pose startPose, double forwardMove, bool calibrate, 
				  double touchVelocity, bool doubleTouch)
{
  // Move to startPose
  cerr << "Move to start pose: " << startPose << endl;
  moveToPose(startPose);

  // Calibrate the force sensor
  if (calibrate) {
    cerr << "Calibrating force sensor" << endl;
    TLU::calibrateForceSensor();
  }
  cerr << "Probing forward" << endl;
  TLU::TouchStatus touchStatus = probeForward(forwardMove, touchVelocity);

  // Return to start position
  if (status.touched) { // If touched, back straight out a bit, first
    cerr << "Back up: " << endl;
    if(doubleTouch){
      cerr << "Double Touch" << endl;
      moveToRelativePose(Pose(0, 0, -0.01, 0, 0, 0));
      touchStatus = probeForward(0.015, 0.002);
    }
    moveToRelativePose(Pose(0, 0, -0.01, 0, 0, 0));
  }

  

  cerr << "Return to start pose from: " << status.eePose << endl;
  moveToPose(startPose);
  return touchStatus;
}

/*
 * Measures the object using the ranged distance sensor
 */
TLU::TouchStatus TLU::measurePoint()
{
  IPC_listenWait(100);
  TLU::TouchStatus measureStatus;
  double dist = readDistanceProbe();

  measureStatus.touched = dist < 250;
  std::cout << "Status: " << status.eePose << std::endl;
  std::cout << status.eeGoalPose << std::endl;
  // std::cout << status.jointAngles.data[0] << std::endl;
  measureStatus.touchPose = status.eePose * Pose(0,0,dist/1000,0,0,0);
  // touchStatus.touchPose = status.eeEndPose;
  return measureStatus;
}

/*
 * Probes forward (in the Z direction) up to maxDist forward at rate of speed
 */
TLU::TouchStatus TLU::probeForward(double maxDist, double speed)
{
  if(speed > 0.01)
    {
      userInput("The touch speed is fast. Continue? (ctrl-C to exit)");
    } 
  // Move in guarded-move velocity mode,
  //  but stop if travelled too far without touching
  Pose eeRelPose(0, 0, maxDist, 0, 0, 0);
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
			    (char *)"XYZ", maxDist, LessConstraint,
			    true, 0.0, status.eePose);
  constraintList.addConstraint(poseConstraint);
  //cerr << poseConstraint << endl;
  IPC_publishData(SET_HARD_CONSTRAINTS_MSG, &constraintList);

  cerr << "Guarded move: " << endl;
  POSE_6DOF_TYPE eeVels(0, 0, speed, 0, 0, 0);
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
  
  TLU::TouchStatus touchStatus;
  touchStatus.touched = status.touched;
  touchStatus.touchPose = status.eeEndPose;

  
  return touchStatus;
}

void TLU::calibrateForceSensor()
{
    sleep(1); // Let things settle down first
    int tmp = 1; // Dummy
    status.calibrated = false;
    IPC_publishData(FORCE_RECALIBRATION_MSG, &tmp);
    do {
      IPC_listenWait(100);
      // cerr << "Waiting for calibration" << endl;
    } while (!status.calibrated);
    cerr << "Force noise: " << status.forceNoise << endl;
}


void TLU::updateTouchStats (const Pose &pose, int num,
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


double TLU::angleDiff(const NDofJointData ang1, const NDofJointData ang2)
{
    // status.jointAngles = ccStatus->manipStatus.cur.position[0];
    double maxDiff = 0;
    for(int i=0; i<7; i++){
      // std::cout << "ang1: " << ang1.data[i] << std::endl;
      // std::cout << "ang2: " << ang2.data[i] << std::endl;
      maxDiff = std::max(maxDiff, std::abs(ang1.data[i] - ang2.data[i]));
    }
    return maxDiff;
}

/*
 *  L2 norm between the poses
 *  If this is near zero then the two poses are the same
 */
double TLU::PoseDiff (const Pose &pose1, const Pose &pose2)
{
  ColVector diffQuat = quaternionError(matrixToQuaternion(pose1.R()),
				       matrixToQuaternion(pose2.R()));
  double sumdiff = (SQ(pose1.x() - pose2.x()) + SQ(pose1.y() - pose2.y()) +
		    SQ(pose1.z() - pose2.z()) +
		    SQ(diffQuat[1]) + SQ(diffQuat[2]) + SQ(diffQuat[3]));
  //cerr << "PoseDiff: " << sqrt(sumdiff) << endl;
  return sqrt(sumdiff);
}

/*
 * Prompts user for input.
 * Currently doesn't do anything with it
 */
void TLU::userInput (const char *msg)
{
  char buff[80];
  cerr << msg << ": ";
  fgets(buff, 80, stdin);
}

