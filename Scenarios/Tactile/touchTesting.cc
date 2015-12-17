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
  const int numTrials = 1;
  double touchDist[numTrials];
  TLU::ipcInit();

  Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  double dtouch = 0.05;
  Pose touchUp(0, 0, dtouch, 0, 0, 0);
  Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  Pose touchSide(0, dtouch, 0, 0, 0, 0);
  Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  bool touched1, touched2, touched3;

  Pose startPose = rotate*touchBase; 
  
  for(int i=0; i < numTrials; i++){
    TLU::TouchStatus tstatus = TLU::touchPoint(startPose, 0.1, (i==0), .005);
    touchDist[i] = (startPose.inverse() * tstatus.touchPose).z();
  }



  
  cerr << "Touches" << endl;
  for(int i = 0; i < numTrials; i++){
    cerr << touchDist[i] << endl;
  }
  //  TLU::guardedMoveToPose(BackOff1);
}
