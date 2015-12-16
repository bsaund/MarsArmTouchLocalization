
#include <unistd.h>
#include <ipc/ipc.h>
#include "Components/Controllers/CoordinatedController/coordinatedController-ipc.h"
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include "Common/Constraint.xdr.h"
#include "Common/ConstrainedMove.xdr.h"
#include "Common/ConstrainedMoveMessages.h"
#include "tactileLocalizationUtils.h"


static TLU::Status status;

//---------------------------------------------
//---Message Handlers from controller----------
//---------------------------------------------

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




//---------------------------------------------
//---Init Function, call at beginning----------
//---------------------------------------------



void TLU::ipcInit ()
{
  IPC_connect("tactileLocalize");

  IPC_subscribeData(FORCE_SENSOR_NOISE_MSG, forceSensorNoiseHnd, NULL);
  IPC_subscribeData(CC_STATUS_MSG, statusHnd, NULL);
  //IPC_subscribeData(CONSTRAINT_VIOLATED_MSG, constraintViolatedHnd, NULL);
  IPC_subscribeData(CONSTRAINED_MOVE_DONE, constrainedMoveHnd, NULL);
}

//---------------------------------------------
//----Useful Functions-------------------------
//---------------------------------------------

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


/* 
 * Moves from startPose forward (in the End Effector Z direction)
 * by forwardMove, stopping if the sensor touches something
 * Optionally calibrate the force sensor
 * Returns a struct of the location of the touch point, and a boolean if contact was made
 */
TLU::TouchStatus TLU::touchPoint (Pose startPose, double forwardMove, bool calibrate)
{
  // Move to startPose
  cerr << "Move to start pose: " << startPose << endl;
  moveToPose(startPose);

  // Calibrate the force sensor
  if (calibrate) {
    TLU::calibrateForceSensor();
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

  TLU::TouchStatus touchStatus;
  touchStatus.touched = status.touched;
  touchStatus.touchPose = status.eeEndPose;

  ConstraintsType hard = HARD_CONSTRAINTS;
  IPC_publishData(REMOVE_CONSTRAINTS_MSG, &hard);

  // Return to start position
  if (status.touched) { // If touched, back straight out a bit, first
    cerr << "Back up: " << endl;
    moveToRelativePose(Pose(0, 0, -0.01, 0, 0, 0));
  }

  cerr << "Return to start pose from: " << status.eePose << endl;
  moveToPose(startPose);
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

