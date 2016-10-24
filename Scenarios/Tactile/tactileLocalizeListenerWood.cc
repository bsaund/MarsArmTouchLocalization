/*****************************************************************
 *
 * Test program for tactile localization using the MARS arm and 
 * JR3 force sensor
 *
 *****************************************************************/

#include <unistd.h>
#include <cmath>
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include <ipc/ipc.h>
#include "tactileLocalizationUtils.h"
#include "tactileLocalizeMsg.h"
#include "moveAroundRealPart.h"

enum GoalFaces {top, front, frontRight, side, bottom};

static void addObservationRos(Pose pose)
{
  double obs[3] = {pose.x(), pose.y(), pose.z()};
  cout << "obs: " << obs[0] << ' ' << obs[1] << ' ' << obs[2] << endl;

  TouchObservation obsMsg = {obs[0], obs[1], obs[2], pose.rz(), pose.ry(), pose.rx()};
  IPC_publishData(TOUCH_OBSERVATION_MSG, &obsMsg);
}

static void sendMoveFinished(char *processName)
{
  ProcessFinished pMsg = {processName};
  cout << "Move Finished: " << processName << endl;
  IPC_publishData(PROCESS_FINISHED_MSG, &pMsg);
}


static TLU::TouchStatus touchPoint(Pose startPose, bool calibrate)
{
  return TLU::touchPoint(startPose, 0.1, calibrate, 0.01, true);
}

static TLU::TouchStatus measurePoint(Pose startPose)
{
  TLU::moveToPose(startPose);
  return TLU::measurePoint();
}

static bool newPose;
static Pose touchPose;

static void touchHnd(MSG_INSTANCE msg, void *callData,
			    void* clientData)
{
  TouchLocation* l = (TouchLocation *)callData;
  Pose startPose(l->x, l->y, l->z, l->r, l->p, l->yaw);
  // std::cout << "xyz: " << l->x << ", " << l->y << ", " << l->z << std::endl;
  // std::cout << "rpy: " << l->r << ", " << l->p << ", " << l->yaw << std::endl;
  // touchPoint(startPose, true);
  touchPose = startPose;
  // std::cout << "Freeing data" << std::endl;
  newPose = true;
  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}


static void checkFace(Pose pose, GoalFaces &face){
  // std::cout << "Pose x, y ,z: " << pose.x() << ", " << pose.y();
  // std::cout << ", " << pose.z() << ", " << std::endl;
  // std::cout << "Pose rx, ry ,rz: " << pose.rx() << ", " << pose.ry();
  // std::cout << ", " << pose.rz() << ", " << std::endl;
  // std::cout << "Next measurement is on ";
  Pose R(0,0,0,pose.rx(), pose.ry(), pose.rz());
  Pose z(0,0,1,0,0,0);
  Pose Rz = R*z;
  

  if(Rz.x() < -.8){
    std::cout << "Front Face\n" << std::endl;
    face = front;
    return;
  }

  if(Rz.y() < -.8){
    std::cout << "Side Face\n" << std::endl;
    face = side;
    return;
  }
  if(pose.z() < -.8){
    std::cout << "Bottom Face\n" << std::endl;
    face = bottom;
    return;
  }
  
  std::cout << "UNKNOWN FACE!!!!\n";
  // std::cout << "Side Face" << std::endl;
  // face = side;
}

static void prepareEE(Pose pose)
{
  GoalFaces face;
  checkFace(touchPose, face);
  switch(face){
  case bottom:
    moveStartToBottom();
    break;
  // case frontRight:
  //   moveTopToFrontRight();
  //   break;
  case side:
    moveStartToSide();
    break;
  }
}

static void returnEE(Pose pose)
{
  GoalFaces face;
  checkFace(touchPose, face);
  switch(face){
  // case front:
  //   moveFrontToTop();
  //   break;
  // case frontRight:
  //   moveFrontRightToTop();
  //   break;
  case side:
    moveSideToStart();
    break;
  case bottom:
    moveBottomToStart();
    break;
  }
}



int main ()
{ 
  TLU::ipcInit();
  IPC_subscribeData(TOUCH_LOCATION_MSG, touchHnd, NULL);
  newPose = false;
  bool calibrate = true;

  TLU::TouchStatus mstatus;

  // moveToStartPose();
  // for(int i=1; i<100; i++){
  //   sleep(1);
  //   // double dist = TLU::readDistanceProbe();
  //   mstatus = measurePoint(touchPose);
  //   std::cout << mstatus.touchPose << std::endl;
  // }
  // return 1;



  moveToStartPose();
  // moveTopToSide();
  // moveSideToTop();
  // moveTopToFrontRight();
  // moveFrontRightToTop();

  // return 1;
 

  
  while(true){

    int i = 0;
    do {
      IPC_listenWait(100);
      i++;
      if(i > 100){
	cerr << "Waiting for Command..." << endl;
	i = 0;
      }
    } while (!newPose);
    newPose = false;
    // TLU::TouchStatus tstatus;
    TLU::TouchStatus mstatus;

    // GoalFaces face;
    // checkFace(touchPose, face);

    prepareEE(touchPose);

    //Change these to switch between Arduino Range Sensor
    //And JR3 Arduino touch sensor
    // mstatus = touchPoint(touchPose, calibrate);
    mstatus = measurePoint(touchPose);
    // ROS_INFO("Touched point at %f, %f, %f", mstatus.touchPose[0], 
    // 	     mstatus.touchPose[1], mstatus.touchPose[2]);
    addObservationRos(mstatus.touchPose);

    returnEE(touchPose);

    sendMoveFinished("movement");

    calibrate = true;  //Calibrate every time because we are changing poses

    
  }

  return 1;
}
