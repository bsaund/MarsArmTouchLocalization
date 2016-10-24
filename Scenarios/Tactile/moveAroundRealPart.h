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

static void moveStartToBottom()
{
  std::cout << "Moving to: Bottom Face" << std::endl;
  moveToStartPose();

  double midAngles[7] = {-1, -.13, .5, 1.4, 1, 1.3, 2.8};
  TLU::moveToAngles(midAngles);

  TLU::moveToPose(Pose(.72, -.09, .5, -3, 0, -.5));
}

static void moveBottomToStart()
{
  TLU::moveToPose(Pose(.72, -.09, .5, -3.14, 0, -.5));

  double midAngles[7] = {-1, -.13, .5, 1.4, 1, 1.3, 2.8};
  TLU::moveToAngles(midAngles);
  
  moveToStartPose();
}

static void moveStartToSide()
{
  std::cout << "Moving to: Side Face" << std::endl;
  moveToStartPose();
 
  double midAngles[7] = {0, -.785, 0, 1.3708, 0, .985, -2.3};
  TLU::moveToAngles(midAngles);
}

static void moveSideToStart()
{
  double midAngles[7] = {0, -.785, 0, 1.3708, 0, .985, -2.3};
  TLU::moveToAngles(midAngles);
  moveToStartPose();
}

static void moveFrontRightToTop()
{
  TLU::moveToPose(Pose(.237, -.336, .36, 1.5708, 0, -2.39));
  TLU::moveToPose(Pose(.237, -.336, .534, 1.5708, 0, -2.39));
  double midAngles[7] = {-1.0, -1.06, 0, 1.62, 0, .64, -.78};
  TLU::moveToAngles(midAngles);
  
  moveToStartPose();
}


#endif
