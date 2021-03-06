#ifndef TACTILCE_LOCALIZATION_UTILS
#define TACTILCE_LOCALIZATION_UTILS

//Touch Localization Utils
namespace TLU{
  // L2 norm between the poses
  double PoseDiff (const Pose &pose1, const Pose &pose2);
  double angleDiff(const NDofJointData ang1, const NDofJointData ang2);

  /* int fd; */
  /* char *portNames; */


  struct Status
  {
    Status () : calibrated(false), moveDone(false), touched(false) {}

    NDofJointData jointAngles;
    NDofJointData jointVels;
    Pose eePose;
    Pose eeGoalPose;
    Pose eeEndPose;
    Pose forceMM;
    Pose forceNoise;
    bool calibrated;
    bool moveDone;
    bool touched;
  };

  struct TouchStatus
  {
    TouchStatus () : touched(false) {}
    bool touched;
    Pose touchPose;
  };

  void ipcInit();

  void updateTouchStats (const Pose &pose, int num,
			 ColVector &mean, ColVector &std);

  TouchStatus touchPoint (Pose startPose, double forwardMove, bool calibrate);
  TouchStatus touchPoint (Pose startPose, double forwardMove, bool calibrate, 
			  double touchVelocity, bool doubleTouch);

  TouchStatus measurePoint();

  TouchStatus probeForward(double maxDist, double speed);

  bool guardedMoveToPose (Pose pose);

  bool moveToRelativePose (Pose relPose);

  bool moveToPose (Pose pose);

  bool moveToAngles (double jointAngles[7]);

  void calibrateForceSensor();

  void userInput(const char *msg);

  double readDistanceProbe();


}










#endif
