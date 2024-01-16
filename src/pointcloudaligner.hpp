#ifndef POINTCLOUDALIGNER_HPP
#define POINTCLOUDALIGNER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
// #include "../source/Cpp/rainbow_table.hpp"
#include "data_types.hpp"
enum class COLORSCHEME { RAINBOW };

class PointCloudAligner {
private:
  // Eigen::Matrix4f m_tofIntrinsic;
  Eigen::Affine3f m_tofIntrinsic;
  TimeStampedPoints m_pointcloud;
  TimeStampedPose m_odomInWorldFrame;
  TimeStampedPose m_gimbalInBodyFrame;
  TimeStampedPose m_tofInGimbalFrame;

  float m_minDistanceClamping;
  float m_maxDistanceClamping;
  COLORSCHEME m_colorScheme;

  // Define the downsampling factor (e.g., keep every nth point)
  int m_numberOfPointTodecimate;

  // Create a matrix to hold the downsampled points
  TimeStampedPoints m_decimatedPoints;

  Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> matrix8bit;

public:
  void clamping();
  void decimation();
  void transformPointFromLidarToWorld();

  PointCloudAligner(
      const Eigen::Affine3f &tofIntrinsic = Eigen::Affine3f::Identity(),
      float minDistanceClaming = 0.50, float maxDistanceClaming = 4.0,
      COLORSCHEME colorScheme = COLORSCHEME::RAINBOW,
      int numberOfPointTodecimate = 5)

      : m_tofIntrinsic(tofIntrinsic), m_minDistanceClamping(),
        m_maxDistanceClamping(minDistanceClaming), m_colorScheme(colorScheme),
        m_numberOfPointTodecimate(numberOfPointTodecimate)

  {}

  TimeStampedPoints getPointCloudInWorldCoordinate();
  Eigen::Affine3f tofIntrinsic() const;
  void setTofIntrinsic(const Eigen::Affine3f &newTofIntrinsic);
  float minDistanceClamping() const;
  void setMinDistanceClamping(float newMinDistanceClamping);
  float maxDistanceClamping() const;
  void setMaxDistanceClamping(float newMaxDistanceClamping);
  COLORSCHEME colorScheme() const;
  void setColorScheme(COLORSCHEME newColorScheme);
  int numberOfPointTodecimate() const;
  void setNumberOfPointTodecimate(int newNumberOfPointTodecimate);
  int getNumberOfPointTodecimate();
  TimeStampedPoints pointcloud() const;
  void setPointcloud(TimeStampedPoints newPointcloud);
  TimeStampedPose tofInGimbalFrame() const;
  void setTofInGimbalFrame(const TimeStampedPose &newTofInGimbalFrame);
  TimeStampedPose odomInWorldFrame() const;
  void setOdomInWorldFrame(const TimeStampedPose &newOdomInWorldFrame);
  TimeStampedPose gimbalInBodyFrame() const;
  void setGimbalInBodyFrame(const TimeStampedPose &newGimbalInBodyFrame);
  TimeStampedPoints getDecimatedPoints() const;
};

#endif // POINTCLOUDALIGNER_HPP
