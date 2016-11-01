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

  TLU::moveToPose(Pose(.72, -.09, .40, -3.14, 0, -.8));
}

static void moveBottomToStart()
{
  TLU::moveToPose(Pose(.72, -.09, .5, -3.14, 0, -.8));

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


static void moveStartToFront()
{
  std::cout << "Moving to: Side Face" << std::endl;
  moveToStartPose();
  double midAngles[7] = {0, -.785, 0, 1.3708, 0, .985, -0.784}; 

  TLU::moveToAngles(midAngles);
}

static void moveFrontToStart()
{
  double midAngles[7] = {0, -.785, 0, 1.3708, 0, .985, -0.784};
  TLU::moveToAngles(midAngles);
  moveToStartPose();
}



#endif
