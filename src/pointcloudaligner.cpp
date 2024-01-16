#include "pointcloudaligner.hpp"

Eigen::Affine3f PointCloudAligner::tofIntrinsic() const {
  return m_tofIntrinsic;
}

void PointCloudAligner::setTofIntrinsic(
    const Eigen::Affine3f &newTofIntrinsic) {
  m_tofIntrinsic = newTofIntrinsic;
}

float PointCloudAligner::minDistanceClamping() const {
  return m_minDistanceClamping;
}

void PointCloudAligner::setMinDistanceClamping(float newMinDistanceClamping) {
  m_minDistanceClamping = newMinDistanceClamping;
}

float PointCloudAligner::maxDistanceClamping() const {
  return m_maxDistanceClamping;
}

void PointCloudAligner::setMaxDistanceClamping(float newMaxDistanceClamping) {
  m_maxDistanceClamping = newMaxDistanceClamping;
}

COLORSCHEME PointCloudAligner::colorScheme() const { return m_colorScheme; }

void PointCloudAligner::setColorScheme(COLORSCHEME newColorScheme) {
  m_colorScheme = newColorScheme;
}

int PointCloudAligner::numberOfPointTodecimate() const {
  return m_numberOfPointTodecimate;
}

void PointCloudAligner::setNumberOfPointTodecimate(
    int newNumberOfPointTodecimate) {
  m_numberOfPointTodecimate = newNumberOfPointTodecimate;
}

int PointCloudAligner::getNumberOfPointTodecimate() {
  return m_numberOfPointTodecimate;
}

TimeStampedPoints PointCloudAligner::pointcloud() const { return m_pointcloud; }

void PointCloudAligner::setPointcloud(TimeStampedPoints newPointcloud) {
  m_pointcloud = newPointcloud;
}

TimeStampedPose PointCloudAligner::tofInGimbalFrame() const {
  return m_tofInGimbalFrame;
}

void PointCloudAligner::setTofInGimbalFrame(
    const TimeStampedPose &newTofInGimbalFrame) {
  m_tofInGimbalFrame = newTofInGimbalFrame;
}

TimeStampedPose PointCloudAligner::odomInWorldFrame() const {
  return m_odomInWorldFrame;
}

void PointCloudAligner::setOdomInWorldFrame(
    const TimeStampedPose &newOdomInWorldFrame) {
  m_odomInWorldFrame = newOdomInWorldFrame;
}

TimeStampedPose PointCloudAligner::gimbalInBodyFrame() const {
  return m_gimbalInBodyFrame;
}

void PointCloudAligner::setGimbalInBodyFrame(
    const TimeStampedPose &newGimbalInBodyFrame) {
  m_gimbalInBodyFrame = newGimbalInBodyFrame;
}

TimeStampedPoints PointCloudAligner::getDecimatedPoints() const {
  return m_decimatedPoints;
}

void PointCloudAligner::clamping() { // matrix (each column is a point)

  // std::cout << "clamping" << std::endl;

  // Iterate over columns and clamp points
  for (int i = 0; i < m_decimatedPoints.pointsMatrix.cols(); ++i) {
    double norm = m_decimatedPoints.pointsMatrix.col(i).norm();

    // std::cout << "norm" << norm << std::endl;

    if (norm < m_minDistanceClamping || norm > m_maxDistanceClamping) {
      m_decimatedPoints.pointsMatrix.col(i).setZero();
    }
  }

  // Print the modified matrix
  // std::cout << "Modified Points Matrix:\n" <<
  // m_decimatedPoints.pointsMatrix.cols() << std::endl;
}

void PointCloudAligner::decimation() {

  /*
    std::cout << "m_pointcloud.pointsMatrix.cols():"
              << m_pointcloud.pointsMatrix.cols() << std::endl;*/

  int decimatedSize =
      (m_pointcloud.pointsMatrix.cols() + m_numberOfPointTodecimate - 1) /
      m_numberOfPointTodecimate;

  /*
    std::cout << "decimatedSize:" << decimatedSize << std::endl;
  */
  m_decimatedPoints.pointsMatrix.resize(3, decimatedSize);

  /*
    std::cout << "rows: " << m_decimatedPoints.pointsMatrix.rows() << std::endl;
    std::cout << "cols: " << m_decimatedPoints.pointsMatrix.cols() << std::endl;
  */
  // Copy every nth point
  for (int i = 0, j = 0; i < m_pointcloud.pointsMatrix.cols();
       i += m_numberOfPointTodecimate, ++j) {
    // std::cout << "i:" << i << std::endl;
    m_decimatedPoints.pointsMatrix.col(j) = m_pointcloud.pointsMatrix.col(i);
  }

  // Now decimatedPoints contains the decimated point cloud
  // std::cout << "Decimated Points Matrix:\n" <<
  // m_decimatedPoints.pointsMatrix.cols() << std::endl;
  // m_decimatedPoints.print();
  return;
}

// void PointCloudAligner::decimation2() {

//   int originalCols = m_pointcloud.pointsMatrix.cols();
//   int decimatedCols = (originalCols + m_numberOfPointTodecimate - 1) /
//                       m_numberOfPointTodecimate;
//   /*
//     std::cout << "originalCols: " << originalCols << std::endl;
//     std::cout << "decimatedCols: " << decimatedCols << std::endl;
//   */
//   // Resize the decimatedPointsMatrix to the correct size
//   m_decimatedPoints.pointsMatrix.resize(3, decimatedCols);

//   // Use Eigen::Map with Eigen::OuterStride to create a view of the decimated
//   // points
//   Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>, 0,
//   Eigen::OuterStride<>>
//       decimatedView(m_pointcloud.pointsMatrix.data(), 3, decimatedCols,
//                     Eigen::OuterStride<>(m_numberOfPointTodecimate));

//   // Copy the data from the view to m_decimatedPoints.pointsMatrix
//   m_decimatedPoints.pointsMatrix = decimatedView;

//   /*
//   std::cout << "m_decimatedPoints.pointsMatrix.cols(): "
//             << m_decimatedPoints.pointsMatrix.cols() << std::endl;
//   */
// }

void PointCloudAligner::transformPointFromLidarToWorld() {

  /*

   pointInWorld=boday_in_local_frame *  gimbal_in_body * tof_in_gimbal *
   tofIntrinsic*pointcloudpoint;


      pointcloud from transformed by intrinsic (fixed)
      Tof in gimbal (fixed)
      gimbal in body frame (dynamic)
      boday in local frame (dynamic)


      # XYZW
      gimbal_in_body=Pose(
      r=(0.188, 0.03285, 0.0019),
      q=(0, 0, 0, 1),  # Dynamic
      ),


      tof_in_gimbal=Pose(
      r=(0.008997, -0.0234, -0.016215),
      q=(0.7071068, 0, 0.7071068, 0),  # XYZW
      ),
   */

  Eigen::Affine3f gimbalPoseInBodyFrame =
      Eigen::Translation3f(m_gimbalInBodyFrame.pose.positionX,
                           m_gimbalInBodyFrame.pose.positionY,
                           m_gimbalInBodyFrame.pose.positionZ) *
      Eigen::Quaternionf(m_gimbalInBodyFrame.pose.orientationW,
                         m_gimbalInBodyFrame.pose.orientationX,
                         m_gimbalInBodyFrame.pose.orientationY,
                         m_gimbalInBodyFrame.pose.orientationZ);

  /*
    std::cout << "gimbalPoseInBodyFrame: \n"
              << gimbalPoseInBodyFrame.matrix() << std::endl;
  */
  Eigen::Affine3f odomPoseInWorld =
      Eigen::Translation3f(m_odomInWorldFrame.pose.positionX,
                           m_odomInWorldFrame.pose.positionY,
                           m_odomInWorldFrame.pose.positionZ) *
      Eigen::Quaternionf(m_odomInWorldFrame.pose.orientationW,
                         m_odomInWorldFrame.pose.orientationX,
                         m_odomInWorldFrame.pose.orientationY,
                         m_odomInWorldFrame.pose.orientationZ);
  /*
    std::cout << "odomPoseInWorld: \n" << odomPoseInWorld.matrix() << std::endl;
  */
  Eigen::Affine3f tofInGimbalFrame =
      Eigen::Translation3f(m_tofInGimbalFrame.pose.positionX,
                           m_tofInGimbalFrame.pose.positionY,
                           m_tofInGimbalFrame.pose.positionZ) *
      Eigen::Quaternionf(m_tofInGimbalFrame.pose.orientationW,
                         m_tofInGimbalFrame.pose.orientationX,
                         m_tofInGimbalFrame.pose.orientationY,
                         m_tofInGimbalFrame.pose.orientationZ);
  /*
    std::cout << "tofInGimbalFrame: \n" << tofInGimbalFrame.matrix() <<
    std::endl;
  */
  Eigen::Affine3f finalTransformation = odomPoseInWorld *
                                        gimbalPoseInBodyFrame *
                                        tofInGimbalFrame * m_tofIntrinsic;

  /*
    std::cout << "finalTransformation: \n"
              << finalTransformation.matrix() << std::endl;
  */

  Eigen::Vector3f res_translation = finalTransformation.translation();
  Eigen::Quaternionf res_quaternion(finalTransformation.rotation());

  /*
    std::cout << "Resulting Pose Translation: " << res_translation.transpose()
              << std::endl;
    std::cout << "Resulting Pose Quaternion: " << res_quaternion.w() << " "
              << res_quaternion.x() << " " << res_quaternion.y() << " "
              << res_quaternion.z() << std::endl;
  */

  // Loop through each column and apply the transformation
  for (int i = 0; i < m_decimatedPoints.pointsMatrix.cols(); ++i) {
    // Extract the point as a 3D vector
    Eigen::Vector3f point = m_decimatedPoints.pointsMatrix.col(i);

    // Apply the transformation
    Eigen::Vector3f transformedPoint = finalTransformation * point;

    // Store the transformed point back into the matrix
    if (transformedPoint.norm() > 20) {
      std::cout << "i: " << i << "transformed norm: " << transformedPoint.norm()
                << std::endl;

      std::cout << "odomPoseInWorld :\n " << odomPoseInWorld.matrix()
                << std::endl;

      std::cout << "m_odomInWorldFrame.pose.positionX :\n "
                << m_odomInWorldFrame.pose.positionX << std::endl;
      std::cout << "m_odomInWorldFrame.pose.positionY:\n "
                << m_odomInWorldFrame.pose.positionY << std::endl;
      std::cout << "m_odomInWorldFrame.pose.positionZ :\n "
                << m_odomInWorldFrame.pose.positionZ << std::endl;

      std::cout << "m_odomInWorldFrame.pose.orientation.W :\n "
                << m_odomInWorldFrame.pose.orientationW << std::endl;
      std::cout << "m_odomInWorldFrame.pose.orientationX :\n "
                << m_odomInWorldFrame.pose.orientationX << std::endl;
      std::cout << "m_odomInWorldFrame.pose.orientationY :\n "
                << m_odomInWorldFrame.pose.orientationY << std::endl;
      std::cout << "m_odomInWorldFrame.pose.orientationZ :\n "
                << m_odomInWorldFrame.pose.orientationZ << std::endl;

      // std::cout << "gimbalPoseInBodyFrame :\n "
      //           << gimbalPoseInBodyFrame.matrix() << std::endl;

      // std::cout << "m_tofIntrinsic :\n " << m_tofIntrinsic.matrix()
      //           << std::endl;

      // std::cout << "finalTransformation:\n " << finalTransformation.matrix()
      //           << std::endl;
    }

    if (point.norm() > 20) {
      std::cout << "i: " << i << "point norm: " << point.norm() << std::endl;
    }

    m_decimatedPoints.pointsMatrix.col(i) = transformedPoint;
  }
}

TimeStampedPoints PointCloudAligner::getPointCloudInWorldCoordinate() {
  decimation();
  clamping();
  transformPointFromLidarToWorld();

  // std::cout << "m_decimatedPoints" << m_decimatedPoints.pointsMatrix.cols()
  // << std::endl;

  // m_decimatedPoints.print();

  // std::cout << "-----------------------" << std::endl;
  return m_decimatedPoints;
}
