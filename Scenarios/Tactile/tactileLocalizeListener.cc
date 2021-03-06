/*****************************************************************
 *
 * Test program for tactile localization using the MARS arm and 
 * JR3 force sensor
 *
 *****************************************************************/

#include <unistd.h>
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include <ipc/ipc.h>
#include "tactileLocalizationUtils.h"
#include "tactileLocalizeMsg.h"


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



int main ()
{ 
  TLU::ipcInit();
  IPC_subscribeData(TOUCH_LOCATION_MSG, touchHnd, NULL);
  newPose = false;
  bool calibrate = true;

  while(true){

    do {
      IPC_listenWait(100);
      // cerr << "Waiting for Command" << endl;
    } while (!newPose);
    newPose = false;
    TLU::TouchStatus tstatus;
    tstatus = touchPoint(touchPose, calibrate);
    calibrate = false;
    addObservationRos(tstatus.touchPose);
  }

  return 1;



// -------------------END----------------------------------

  Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  double dtouch = 0.05;
  Pose touchUp(0, 0, dtouch, 0, 0, 0);
  Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  Pose touchSide(0, dtouch, 0, 0, 0, 0);
  Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  bool touched1, touched2, touched3;

  Pose touch1 = rotate*touchBase;

  TLU::TouchStatus tstatus;


  tstatus = touchPoint(touch1, true);
  touch1 = tstatus.touchPose;
  touched1 = tstatus.touched;

  addObservationRos(touch1);

  Pose touch2 = rotate*(touchBase+touchUp);

  tstatus = touchPoint(touch2, false);
  touch2 = tstatus.touchPose;
  touched2 = tstatus.touched;
  addObservationRos(touch2);

  Pose touch3 = rotate*(touchBase+touchUp+touchSide);

  tstatus = touchPoint(touch3, false);
  touch3 = tstatus.touchPose;
  touched3 = tstatus.touched;

  addObservationRos(touch3);


  TLU::moveToPose(rotate*(touchBase+touchUp+touchSide));
  Pose touch4 = rotate*(touchBase+touchSide);

  tstatus = touchPoint(touch4, false);
  addObservationRos(touch4);


  Pose touch5 = rotate*(touchBase+touchMiddle);

  tstatus = touchPoint(touch5, false);
  addObservationRos(touch5);

  // addObservation(pfilter, tstatus.touchPose, tstatus.touched);
  // moveNearPlane(pfilter, 0.005);
  TLU::userInput("Move after 5 observations");

  Pose touch6 = rotate*(touchBase+touchDown);
  tstatus = touchPoint(touch6, false);
  addObservationRos(touch6);
  // addObservation(pfilter, tstatus.touchPose, tstatus.touched);
  // moveNearPlane(pfilter, 0.01);
  TLU::userInput("Move after 6 observations");

  Pose touch7 = rotate*(touchBase+touchDown+touchSide);
  tstatus = touchPoint(touch7, false);
  addObservationRos(touch7);
  // addObservation(pfilter, tstatus.touchPose, tstatus.touched);
  // moveNearPlane(pfilter, 0.01);
  TLU::userInput("Move after 7 observations");


}
