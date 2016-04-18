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

enum GoalFaces {top, front, frontRight, side};

static void addObservationRos(Pose pose)
{
  double obs[3] = {pose.x(), pose.y(), pose.z()};
  cout << "obs: " << obs[0] << ' ' << obs[1] << ' ' << obs[2] << endl;

  TouchObservation obsMsg = {obs[0], obs[1], obs[2], pose.rz(), pose.ry(), pose.rx()};
  IPC_publishData(TOUCH_OBSERVATION_MSG, &obsMsg);
}


static TLU::TouchStatus touchPoint(Pose startPose, bool calibrate)
{
  return TLU::touchPoint(startPose, 0.1, calibrate, 0.01, true);
}

static bool newPose;
static Pose touchPose;

static void touchHnd(MSG_INSTANCE msg, void *callData,
			    void* clientData)
{
  TouchLocation* l = (TouchLocation *)callData;
  Pose startPose(l->x, l->y, l->z, l->r, l->p, l->yaw);
  std::cout << "xyz: " << l->x << ", " << l->y << ", " << l->z << std::endl;
  std::cout << "rpy: " << l->r << ", " << l->p << ", " << l->yaw << std::endl;
  // touchPoint(startPose, true);
  touchPose = startPose;
  std::cout << "Freeing data" << std::endl;
  newPose = true;
  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}

static void moveToStartPose()
{

  std::cout << "Moving to Safe Pose" << std::endl;
  double topAngles[7] = {0, -.785, 0, .785, 0, 1.57, 0};
  TLU::moveToAngles(topAngles);



}

static void moveTopToFront()
{
  std::cout << "Moving to: Front Face" << std::endl;
  moveToStartPose();


  double midAngles[7] = {.25, -.93, 0, 2.26, -2.37, 1.45, 0};
  TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(0.913, -0.128, 0.706, -1.264, -1.396, -2.389));
  TLU::moveToPose(Pose(0.812, -0.050, 0.391, -1.468, -1.396, -2.104));

  
}

static void moveFrontToTop()
{
  TLU::moveToPose(Pose(0.812, -0.050, 0.391, -1.468, -1.396, -2.104));
  TLU::moveToPose(Pose(0.913, -0.128, 0.706, -1.264, -1.396, -2.389));
  double midAngles[7] = {.25, -.93, 0, 2.26, -2.37, 1.45, 0};
  TLU::moveToAngles(midAngles);
  
  moveToStartPose();
}

static void moveTopToFrontRight()
{
  std::cout << "Moving to: Front Face" << std::endl;
  moveToStartPose();


  double midAngles[7] = {-1.5, -.44, 0, 2, 2, 1.6, 0};
  TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(.577, -.611, .534, .283, -1.536, 2.5));
  TLU::moveToPose(Pose(.577, -.611, .347, .398, -1.532, 2.387));

  
}

static void moveFrontRightToTop()
{
  TLU::moveToPose(Pose(.577, -.611, .347, .398, -1.532, 2.387));
  TLU::moveToPose(Pose(.577, -.611, .534, .283, -1.536, 2.5));
  double midAngles[7] = {-1.5, -.44, 0, 2, 2, 1.6, 0};
  TLU::moveToAngles(midAngles);
  
  moveToStartPose();
}

static void moveTopToSide()
{
  std::cout << "Moving to: Side Face" << std::endl;
  moveToStartPose();
  double midAngles[7] = {-.65, -.78, 0, 2, 1.34, 1.65, 0};
  TLU::moveToAngles(midAngles);
  TLU::moveToPose(Pose(.69, .14, .65, 1.58, -1.23, 2.724));
  TLU::moveToPose(Pose(.71, .13, .4, 1.58, -1.23, 2.724));

}

static void moveSideToTop()
{
  std::cout << "Moving from side to top" << std::endl;
  TLU::moveToPose(Pose(.71, .13, .4, 1.58, -1.23, 2.724));
  std::cout << "Moving from side to stop stage 2" << std::endl;
  TLU::moveToPose(Pose(.69, .14, .65, 1.58, -1.23, 2.724));

  double midAngles[7] = {-.65, -.78, 0, 2, 1.34, 1.65, 0};
  TLU::moveToAngles(midAngles);

  moveToStartPose();
}

static void checkFace(Pose pose, GoalFaces &face){
  std::cout << "Pose x, y ,z: " << pose.x() << ", " << pose.y();
  std::cout << ", " << pose.z() << ", " << std::endl;
  std::cout << "Pose rx, ry ,rz: " << pose.rx() << ", " << pose.ry();
  std::cout << ", " << pose.rz() << ", " << std::endl;

  if(std::abs(pose.rx()) > 3){
    std::cout << "Top Face" << std::endl;
    face = top;
    return;
  }

  if(pose.rz() < -1.4){
    std::cout << "Front Face" << std::endl;
    face = front;
    return;
  }
  if(pose.y() < -.4){
    std::cout << "Front Right Face" << std::endl;
    face = frontRight;
    return;
  }

  std::cout << "Side Face" << std::endl;
  face = side;
}

static void prepareEE(Pose pose)
{
  GoalFaces face;
  checkFace(touchPose, face);
  switch(face){
  case front:
    moveTopToFront();
    break;
  case frontRight:
    moveTopToFrontRight();
    break;
  case side:
    moveTopToSide();
    break;
  }
}

static void returnEE(Pose pose)
{
  GoalFaces face;
  checkFace(touchPose, face);
  switch(face){
  case front:
    moveFrontToTop();
    break;
  case frontRight:
    moveFrontRightToTop();
    break;
  case side:
    moveSideToTop();
    break;
  }
}



int main ()
{ 
  TLU::ipcInit();
  IPC_subscribeData(TOUCH_LOCATION_MSG, touchHnd, NULL);
  newPose = false;
  bool calibrate = true;


  moveToStartPose();
  // moveTopToSide();
  // moveSideToTop();
  // moveTopToFrontRight();
  // moveFrontRightToTop();

  // return 1;
 

  
  while(true){

    do {
      IPC_listenWait(100);
      // cerr << "Waiting for Command" << endl;
    } while (!newPose);
    newPose = false;
    TLU::TouchStatus tstatus;

    // GoalFaces face;
    // checkFace(touchPose, face);

    prepareEE(touchPose);

    tstatus = touchPoint(touchPose, calibrate);

    returnEE(touchPose);

    calibrate = true;  //Calibrate every time because we are changing poses
    addObservationRos(tstatus.touchPose);

  }

  return 1;
}
