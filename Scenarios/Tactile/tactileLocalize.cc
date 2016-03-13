/*****************************************************************
 *
 * Test program for tactile localization using the MARS arm and 
 * JR3 force sensor
 *
 *****************************************************************/

#include <unistd.h>
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include "particleFilter.h"
#include <ipc/ipc.h>
#include "tactileLocalizationUtils.h"
#include "tactileLocalizeMsg.h"

static void addObservation (particleFilter &pfilter, Pose pose, bool touched)
{
  if (touched) {
    double obs[3] = {pose.x(), pose.y(), pose.z()};
    cout << "obs: " << obs[0] << ' ' << obs[1] << ' ' << obs[2] << endl;
    pfilter.addObservation(obs);
    particleFilter::cspace X_est[2];
    pfilter.estimatedDistribution(X_est);
    TouchObservation obsMsg = {obs[0], obs[1], obs[2]};
    IPC_publishData("TestTopicMSG", &obsMsg);

    cout << "est: " << X_est[0][0] << ' ' << X_est[0][1] << ' ' << X_est[0][2]
	 << " (" << X_est[1][0] << ")" << endl;
  } else {
    cerr << "NEED TO DO SOMETHING WHEN TARGET IS MISSED\n";
  }
}

static void addObservationRos(Pose pose)
{
  double obs[3] = {pose.x(), pose.y(), pose.z()};
  cout << "obs: " << obs[0] << ' ' << obs[1] << ' ' << obs[2] << endl;
  // pfilter.addObservation(obs);
  // particleFilter::cspace X_est[2];
  // pfilter.estimatedDistribution(X_est);
  TouchObservation obsMsg = {obs[0], obs[1], obs[2]};
  IPC_publishData("TestTopicMSG", &obsMsg);

  // cout << "est: " << X_est[0][0] << ' ' << X_est[0][1] << ' ' << X_est[0][2]
  // 	 << " (" << X_est[1][0] << ")" << endl;
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



static TLU::TouchStatus touchPoint(Pose startPose, bool calibrate)
{
  return TLU::touchPoint(startPose, 0.1, calibrate, 0.01, true);
}


int main ()
{ 
  TLU::ipcInit();

  // addObservationTest();
  // int b >> std::cin;
  // int b;
  // std::cin >> b;
  // return 0;
  
  Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  double dtouch = 0.05;
  Pose touchUp(0, 0, dtouch, 0, 0, 0);
  Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  Pose touchSide(0, dtouch, 0, 0, 0, 0);
  Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  bool touched1, touched2, touched3;

  // particleFilter pfilter(1000);

  Pose touch1 = rotate*touchBase;
  std::cout << "xyz: " << touch1.x() << ", " << touch1.y() << ", " << touch1.z() << std::endl;
  std::cout << "rpy: " << touch1.rx() << ", " << touch1.ry() << ", " << touch1.rz() << std::endl;

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

  // particleFilter::cspace binit[2];

  // computeInitialDistribution(binit, touch1, touch2, touch3);
  // pfilter.setDistribution(binit);

  // addObservation(pfilter, touch1, touched1);
  // addObservation(pfilter, touch2, touched2);
  // addObservation(pfilter, touch3, touched3);

  TLU::moveToPose(rotate*(touchBase+touchUp+touchSide));
  Pose touch4 = rotate*(touchBase+touchSide);

  tstatus = touchPoint(touch4, false);
  addObservationRos(touch4);

  // addObservation(pfilter, tstatus.touchPose, tstatus.touched);
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
