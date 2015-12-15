#ifndef TACTILCE_LOCALIZATION_UTILS
#define TACTILCE_LOCALIZATION_UTILS


namespace TLU{
  // L2 norm between the poses
  double PoseDiff (const Pose &pose1, const Pose &pose2);


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




}










#endif
