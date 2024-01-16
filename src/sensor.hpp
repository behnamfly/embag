#pragma once
#include "data_types.hpp"
class SensorInterface {
private:
public:
  virtual TimeStampedPose getOdometry() = 0;
  virtual TimeStampedPose getMountControlOrientation() = 0;
  virtual TimeStampedPoints getPointCloud() = 0;
};
