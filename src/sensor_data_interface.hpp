#include "wrapper.hpp"
#include <memory>

// SensorDataInterface using the type-erased wrapper classes
class SensorDataInterface {
public:
  virtual void
  addOdometryData(std::shared_ptr<OdometryWrapper> odometryData) = 0;
  virtual void
  addOrientationData(std::shared_ptr<OrientationWrapper> orientationData) = 0;
  virtual void
  addPointCloudData(std::shared_ptr<PointCloudWrapper> pointCloudData) = 0;

  virtual void processOdometry() = 0;
  virtual void processOrientation() = 0;
  virtual void processPointCloud() = 0;
};

class ROSDataInterface : public SensorDataInterface {
  void addOdometryData(std::shared_ptr<OdometryWrapper> odometryData) {}
  void addOrientationData(std::shared_ptr<OrientationWrapper> orientationData) {
  }
  void addPointCloudData(std::shared_ptr<PointCloudWrapper> pointCloudData) {}

  void processOdometry() {}
  void processOrientation() {}
  void processPointCloud() {}
};

class MavlinkDataInterface : public SensorDataInterface {
  void addOdometryData(std::shared_ptr<OdometryWrapper> odometryData) {}
  void addOrientationData(std::shared_ptr<OrientationWrapper> orientationData) {
  }
  void addPointCloudData(std::shared_ptr<PointCloudWrapper> pointCloudData) {}

  void processOdometry() {}
  void processOrientation() {}
  void processPointCloud() {}
};
