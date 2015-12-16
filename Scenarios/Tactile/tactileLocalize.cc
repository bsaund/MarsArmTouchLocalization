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
  TLU::guardedMoveToPose(nearPt);
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
  TLU::ipcInit();


#if 0
  /*  Pose touch1(0.404, 0.303, 1.267, 0.0, M_PI/2, M_PI/4);
  ColVector mean(6), std(6);
  int num = 10;
  for (int i=1; i<=num; i++) {
    touchPoint(touch1, 0.1, (i==1)); // Calibrate force sensor just first time
    updateTouchStats(status.eeEndPose, i, mean, std);
    cerr << "Mean: " << mean << "Std: " << std;
    }*/
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

  TLU::TouchStatus tstatus;
//#define TEST_PF
#ifndef TEST_PF
  tstatus = TLU::touchPoint(touch1, 0.1, true);
  touch1 = tstatus.touchPose;
  touched1 = tstatus.touched;
#else
  touched1 = true;
#endif

  Pose touch2 = rotate*(touchBase+touchUp);
#ifndef TEST_PF
  tstatus = TLU::touchPoint(touch2, 0.1, false);
  touch2 = tstatus.touchPose;
  touched2 = tstatus.touched;
#else
  touched2 = true;
#endif

  Pose touch3 = rotate*(touchBase+touchUp+touchSide);
#ifndef TEST_PF
  tstatus = TLU::touchPoint(touch3, 0.1, false);
  touch3 = tstatus.touchPose;
  touched3 = tstatus.touched;
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

  TLU::moveToPose(rotate*(touchBase+touchUp+touchSide));
#if 0
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 3 observations");
#endif
  Pose touch4 = rotate*(touchBase+touchSide);
#ifndef TEST_PF
  tstatus = TLU::touchPoint(touch4, 0.1, false);
#else
  tstatus.touchPose = touch4; tstatus.touched = true;
#endif
  addObservation(pfilter, tstatus.touchPose, tstatus.touched);
#if 0
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 4 observations");
#endif
  Pose touch5 = rotate*(touchBase+touchMiddle);
#ifndef TEST_PF  
  tstatus = TLU::touchPoint(touch5, 0.1, false);
#else
  tstatus.touchPose = touch5; tstatus.touched = true;
#endif
  addObservation(pfilter, tstatus.touchPose, tstatus.touched);
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 5 observations");
#if 1
  Pose touch6 = rotate*(touchBase+touchDown);
  tstatus = TLU::touchPoint(touch6, 0.1, false);
  addObservation(pfilter, tstatus.touchPose, tstatus.touched);
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 6 observations");

  Pose touch7 = rotate*(touchBase+touchDown+touchSide);
  tstatus = TLU::touchPoint(touch7, 0.1, false);
  addObservation(pfilter, tstatus.touchPose, tstatus.touched);
  moveNearPlane(pfilter, 0.01);
  userInput("Move after 7 observations");
#endif
#endif
}
