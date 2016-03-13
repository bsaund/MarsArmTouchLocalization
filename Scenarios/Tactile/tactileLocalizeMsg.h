#ifndef TACTILE_LOCALIZE_MSG_H
#define TACTILE_LOCALIZE_MSG_H

#include "Common/commonMath.h"

typedef struct {
  double x;
  double y;
  double z;
} TouchObservation;
#define TOUCH_OBSERVATION_FORM "{double, double, double}"
#define TOUCH_OBSERVATION_MSG "TOUCH_OBSERVATION"


typedef struct {
  double x;
  double y;
  double z;
  double r;
  double p;
  double yaw;
} TouchLocation;
#define TOUCH_LOCATION_FORM "{double, double, double, double, double, double}"
#define TOUCH_LOCATION_MSG "TOUCH_LOCATION"



#endif
