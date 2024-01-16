#pragma once
#include "../lib/embag.h"
#include "../lib/view.h"
#include "pointcloudaligner.hpp"
#include "sensor.hpp"
#include <memory>

class ROSProcesser : public SensorInterface {
private:
  TimeStampedPose m_mount_control;
  TimeStampedPose m_odometry;
  TimeStampedPoints m_pointcloud;
  std::unique_ptr<PointCloudAligner> m_aligner_ptr;

public:
  void setPointCloudAligner(std::unique_ptr<PointCloudAligner> aligner_ptr) {
    m_aligner_ptr = std::move(aligner_ptr);
  };
  void processBagData(
      Embag::View &view,
      const std::string &mountControlOrientationTopic =
          "/mavros/mount_control/orientation",
      const std::string &odometryTopic = "/mavros/odometry/in",
      const std::string &pointcloudTopic = "/royale_camera_driver/point_cloud");

  TimeStampedPose getOdometry() override;
  TimeStampedPose getMountControlOrientation() override;
  TimeStampedPoints getPointCloud() override;
};
