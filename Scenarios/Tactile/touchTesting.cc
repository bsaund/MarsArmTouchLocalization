/*****************************************************************
 *
 * Test program for the touch probe on the MARS arm
 *
 *****************************************************************/

#include <unistd.h>
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include "tactileLocalizationUtils.h"

int main()
{
  // const int numTrials = 1;
  // double touchDist[numTrials];
  TLU::ipcInit();

  // Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  // double dtouch = 0.05;
  // Pose touchUp(0, 0, dtouch, 0, 0, 0);
  // Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  // Pose touchSide(0, dtouch, 0, 0, 0, 0);
  // Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  // Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  // bool touched1, touched2, touched3;

  // Pose startPose = rotate*touchBase; 
  
  // for(int i=0; i < numTrials; i++){
  //   TLU::TouchStatus tstatus = TLU::touchPoint(startPose, 0.1, (i==0), .01, true);
  //   touchDist[i] = (startPose.inverse() * tstatus.touchPose).z();
  //   Pose BackOff1 = tstatus.touchPose * Pose(0,0,-.001,0,0,0);
  //   TLU::guardedMoveToPose(BackOff1);
  //   TLU::userInput("Continue?");
  // }

 
  // TLU::guardedMoveToPose(startPose);
  
  // cerr << "Touches" << endl;
  // for(int i = 0; i < numTrials; i++){
  //   cerr << touchDist[i] << endl;
  // }
  //  TLU::guardedMoveToPose(BackOff1);
  double angles[7] = {1.0,-1.4,0,1.37,0,1.5,0};
  // double angles[7] = {0, -1.57, 0, 0 ,0 ,0 ,0};
  TLU::moveToAngles(angles);
  

  // TLU::moveToPose(Pose(.4, .4, .7, M_PI, 0, 0));
  // TLU::moveToPose(Pose(.4, .4, .7, -M_PI, 0, -M_PI/2));
  // // double topAngles[7] = {1.070, -1.346, -0.193, 0.991, 0.046, 1.930, -0.675};
  // double topAngles[7] = {1.025, -1.415, -0.114, 1.370, 0.017, 1.616, -0.658};
  // TLU::moveToAngles(topAngles);

  // double midAngles[7] = {1.557, -1.415, 0.511, 1.370, -1.256, 1.600, 0.3};
  // TLU::moveToAngles(midAngles);

  // // TLU::moveToPose(Pose(.4, .4, .7, -M_PI/2, 0, -M_PI/2));
  // double frontAngles[7] = {1.557, -0.961, 0.511, 2.227, -1.256, 1.966, 1.327};
  // TLU::moveToAngles(frontAngles);
}
