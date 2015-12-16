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
  TLU::ipcInit();

  Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  double dtouch = 0.05;
  Pose touchUp(0, 0, dtouch, 0, 0, 0);
  Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  Pose touchSide(0, dtouch, 0, 0, 0, 0);
  Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  bool touched1, touched2, touched3;

  Pose touch1 = rotate*touchBase;
  TLU::TouchStatus tstatus = TLU::touchPoint(touch1, 0.1, true);

  Pose touchedPose1 = tstatus.touchPose;

  Pose BackOff1 = touchedPose1 * Pose(0,0,-.01,0,0,0);
  
  cerr << "Suggesting: " << BackOff1 << endl;
  TLU::guardedMoveToPose(BackOff1);
}
