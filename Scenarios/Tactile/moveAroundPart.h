#ifndef MOVE_AROUND_PART_H
#define MOVE_AROUND_PART_H

#include "tactileLocalizationUtils.h"
#include "tactileLocalizeMsg.h"



static void moveToStartPose()
{

  std::cout << "Moving to Safe Pose" << std::endl;
  double topAngles[7] = {0, -.785, 0, 1.3708, 0, .985, -0.784};
  TLU::moveToAngles(topAngles);
}

static void moveTopToFront()
{
  std::cout << "Moving to: Front Face" << std::endl;
  moveToStartPose();


  // double midAngles[7] = {.25, -.93, 0, 2.26, -2.37, 1.45, 0};
  double midAngles[7] = {0.0, -.9, 0, 1.7, 0, .81, 0};
  TLU::moveToAngles(midAngles);

  // TLU::moveToPose(Pose(0.863, -0.128, 0.706, -1.264, -1.396, -2.389));
  // TLU::moveToPose(Pose(0.812, -0.050, 0.391, -1.468, -1.396, -2.104));
  TLU::moveToPose(Pose(0.636, 0, .523, 1.5708, 0, -2.39));
  TLU::moveToPose(Pose(0.636, 0, .323, 1.5708, 0, -2.39));

  
}

static void moveFrontToTop()
{
  TLU::moveToPose(Pose(0.636, 0, .33, 1.5708, 0, -2.39));
  TLU::moveToPose(Pose(0.636, 0, .523, 1.5708, 0, -2.39));
  double midAngles[7] = {0.0, -.9, 0, 1.7, 0, .81, 0};
  // TLU::moveToPose(Pose(0.812, -0.050, 0.391, -1.468, -1.396, -2.104));
  // TLU::moveToPose(Pose(0.863, -0.128, 0.706, -1.264, -1.396, -2.389));
  // double midAngles[7] = {.25, -.93, 0, 2.26, -2.37, 1.45, 0};
  TLU::moveToAngles(midAngles);
  
  moveToStartPose();
}

static void moveTopToFrontRight()
{
  std::cout << "Moving to: Front Face" << std::endl;
  moveToStartPose();


  double midAngles[7] = {-1.0, -1.06, 0, 2.02, 0, .64, -.78};
  TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(.237, -.336, .534, 1.5708, 0, -2.39));
  TLU::moveToPose(Pose(.237, -.336, .34, 1.5708, 0, -2.39));

  
}

static void moveFrontRightToTop()
{
  TLU::moveToPose(Pose(.237, -.336, .36, 1.5708, 0, -2.39));
  TLU::moveToPose(Pose(.237, -.336, .534, 1.5708, 0, -2.39));
  double midAngles[7] = {-1.0, -1.06, 0, 1.62, 0, .64, -.78};
  TLU::moveToAngles(midAngles);
  
  moveToStartPose();
}

static void moveTopToSide()
{
  std::cout << "Moving to: Side Face" << std::endl;
  moveToStartPose();

  double midAngles[7] = {0.0, -.9, 0, 1.7, 0, .81, -1.57};
  TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(0.636, 0, .523, 1.5708, 0, -1.0));
  TLU::moveToPose(Pose(0.636, 0, .323, 1.5708, 0, -1.0));

  // TLU::moveToAngles(midAngles);
  // TLU::moveToPose(Pose(.69, .14, .65, 1.58, -1.23, 2.724));
  // TLU::moveToPose(Pose(.71, .13, .4, 1.58, -1.23, 2.724));

}

static void moveSideToTop()
{
  std::cout << "Moving from side to top" << std::endl;
  // TLU::moveToPose(Pose(.71, .13, .4, 1.58, -1.23, 2.724));
  // std::cout << "Moving from side to stop stage 2" << std::endl;
  // TLU::moveToPose(Pose(.69, .14, .65, 1.58, -1.23, 2.724));

  // double midAngles[7] = {0.55, -.9, 0, 1.7, 0, .81, -1.5};
  // TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(0.636, 0, .34, 1.5708, 0, -1.0));
  TLU::moveToPose(Pose(0.636, 0, .523, 1.5708, 0, -1.0));
  double midAngles[7] = {0.0, -.9, 0, 1.7, 0, .81, -1.57};

  TLU::moveToAngles(midAngles);
  


  moveToStartPose();
}

static void moveStartToTop()
{
  std::cout << "Moving to: Top Face" << std::endl;
  moveToStartPose();

  double midAngles[7] = {0.0, -.8, 0, 1.56, -1.59, 1.55, -1.65};
  TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(0.74, -.32, .77, 0, 0, 0));
  TLU::moveToPose(Pose(0.74, -.32, .55, 0,0,0));

}

static void moveTopToStart()
{
  std::cout << "Moving from top to start" << std::endl;
  TLU::moveToPose(Pose(0.74, -.32, .55, 0,0,0));
  TLU::moveToPose(Pose(0.74, -.32, .77, 0, 0, 0));

  double midAngles[7] = {0.0, -.8, 0, 1.56, -1.59, 1.55, -1.65};
  TLU::moveToAngles(midAngles);
  moveToStartPose();
}

#endif
