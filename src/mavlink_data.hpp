
#include "sensor.hpp"

class MavlinkOdometry;
class MavlinkGimbalOrientation;
class MavlinkPointCloud;

class MavlinkData : public SensorInterface {
  void addOdometryData(const MavlinkOdometry &odometryData) {}
  void addOrientationData(const MavlinkGimbalOrientation &orientationData) {}
  void addPointCloudData(const MavlinkPointCloud &pointCloudData) {}

  void processOdometry() override {}
  void processOrientation() override {}
  void processPointCloud() override {}
};