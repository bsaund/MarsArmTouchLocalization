
#include <unistd.h>
#include <ipc/ipc.h>
#include "Components/Controllers/CoordinatedController/coordinatedController-ipc.h"
#include "Components/Controllers/CoordinatedController/utils.h"
#include "Common/commonMath.h"
#include "Common/Constraint.xdr.h"
#include "Common/ConstrainedMove.xdr.h"
#include "Common/ConstrainedMoveMessages.h"
#include "tactileLocalizationUtils.h"



// L2 norm between the poses
double TLU::PoseDiff (const Pose &pose1, const Pose &pose2)
{
  ColVector diffQuat = quaternionError(matrixToQuaternion(pose1.R()),
				       matrixToQuaternion(pose2.R()));
  double sumdiff = (SQ(pose1.x() - pose2.x()) + SQ(pose1.y() - pose2.y()) +
		    SQ(pose1.z() - pose2.z()) +
		    SQ(diffQuat[1]) + SQ(diffQuat[2]) + SQ(diffQuat[3]));
  //cerr << "PoseDiff: " << sqrt(sumdiff) << endl;
  return sqrt(sumdiff);
}


