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

enum GoalFaces {top, front, side};

static void addObservationRos(Pose pose)
{
  double obs[3] = {pose.x(), pose.y(), pose.z()};
  cout << "obs: " << obs[0] << ' ' << obs[1] << ' ' << obs[2] << endl;

  TouchObservation obsMsg = {obs[0], obs[1], obs[2]};
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

static void moveToSafePose()
{
  Pose safePose(.445, .315, .764, -2.76, -.254, -.06);
  std::cout << "Moving to Safe Pose" << std::endl;
  TLU::moveToPose(safePose);
}

static void moveTopToFront()
{
  std::cout << "Moving to: Front Face" << std::endl;
  TLU::moveToPose(Pose(.4, .4, .7, M_PI, 0, 0));
  TLU::moveToPose(Pose(.4, .4, .7, -M_PI, 0, -M_PI/2));
  double topAngles[7] = {1.025, -1.415, -0.114, 1.370, 0.017, 1.616, -0.658};
  TLU::moveToAngles(topAngles);

  double midAngles[7] = {1.557, -1.415, 0.511, 1.570, -.6, 1.600, 0.3};
  TLU::moveToAngles(midAngles);

  double frontAngles[7] = {1.557, -0.961, 0.511, 2.227, -1.256, 1.966, 1.327};
  TLU::moveToAngles(frontAngles);
  
}

static void moveFrontToTop()
{
  TLU::moveToPose(Pose(.4, .4, .7, -M_PI/2, 0, -M_PI/2));
  double frontAngles[7] = {1.557, -0.961, 0.511, 2.227, -1.256, 1.966, 1.327};
  TLU::moveToAngles(frontAngles);

  double midAngles[7] = {1.557, -1.415, 0.511, 1.570, -.6, 1.600, 0.3};
  TLU::moveToAngles(midAngles);

  double topAngles[7] = {1.025, -1.415, -0.114, 1.370, 0.017, 1.616, -0.658};
  TLU::moveToAngles(topAngles);

  TLU::moveToPose(Pose(.4, .4, .7, M_PI, 0, 0));
}

static void moveTopToSide()
{
  std::cout << "Moving to: Side Face" << std::endl;
  TLU::moveToPose(Pose(.53, .3, .7, -M_PI, 0, 0));
  // double topAngles[7] = {0.698, -1.344, 0.112, 1.288, 0.025, 1.628, -2.55};
  // TLU::moveToAngles(topAngles);

  double midAngles[7] = {-.25, -1.344, -0.112, 1.388, 0.6, 1.628, -1.8};
  TLU::moveToAngles(midAngles);


  double sideAngles[7] = {-0.244, -0.959, -0.187, 2.139, 1.237, 1.815, -1.2};
  TLU::moveToAngles(sideAngles);


  // TLU::moveToPose(Pose(.53, .3, .7, -M_PI/2, 0, 0));
}

static void moveSideToTop()
{
  // TLU::moveToPose(Pose(.53, .3, .7, -M_PI/2, 0, 0));
  double midAngles[7] = {-.25, -1.344, -0.112, 1.388, 0.6, 1.628, -1.8};
  TLU::moveToAngles(midAngles);

  double topAngles[7] = {.593, -1.345, 0.021, 1.29, 0, 1.627, -2.529};
  TLU::moveToAngles(topAngles);


  // TLU::moveToPose(Pose(.53, .3, .7, -M_PI, 0, 0));
}

static void checkFace(Pose pose, GoalFaces &face){
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


  // moveToSafePose();
  while(true){

    do {
      IPC_listenWait(100);
      // cerr << "Waiting for Command" << endl;
    } while (!newPose);
    newPose = false;
    TLU::TouchStatus tstatus;


    prepareEE(touchPose);
    // moveTopToFront();
    tstatus = touchPoint(touchPose, calibrate);
    // moveFrontToTop();
    returnEE(touchPose);

    calibrate = true;  //Calibrate every time because we are changing poses
    addObservationRos(tstatus.touchPose);
    // moveToSafePose();
  }

  return 1;
}
