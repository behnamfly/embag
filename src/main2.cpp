
#include "data_types.hpp"
#include <algorithm>
#include <iostream>
#include <utility>

#include "pointcloudaligner.hpp"

#include "circular_buffer.hpp"
#include <Eigen/Dense>
#include <any>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


void clamping() { // Example matrix (each column is a point)
  Eigen::MatrixXf pointsMatrix(3, 3);
  pointsMatrix << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  // Define min and max distance
  double minDistance = 2.0;
  double maxDistance = 8.5;

  // Iterate over columns and clamp points
  for (int i = 0; i < pointsMatrix.cols(); ++i) {
    // Eigen::Vector3d point = pointsMatrix.col(i);
    double norm = pointsMatrix.col(i).norm();
    if (norm < minDistance || norm > maxDistance) {
      pointsMatrix.col(i).setZero();
    }
  }

  // Print the modified matrix
  std::cout << "Modified Points Matrix:\n" << pointsMatrix << std::endl;
}

int decimation() {
  std::string line;
  std::ifstream file("bunny.csv"); // Replace with your CSV file path

  std::vector<Eigen::Vector3f> pointsList;

  // Check if file is open
  if (file.is_open()) {
    while (getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      Eigen::Vector3f point;
      int i = 0;

      // Parse each line
      while (getline(ss, value, ',')) {
        point[i++] = std::stof(value);
      }

      pointsList.push_back(point);
    }
    file.close();
  } else {
    std::cerr << "Unable to open file" << std::endl;
    return 1;
  }

  // Create and populate the Eigen matrix
  Eigen::Matrix<float, 3, Eigen::Dynamic> points;
  points.resize(3, pointsList.size());

  for (size_t i = 0; i < pointsList.size(); ++i) {
    points.col(i) = pointsList[i];
  }

  // Print the matrix
  // std::cout << "Points Matrix:\n" << points << std::endl;

  std::cout << "points.rows():\n" << points.rows() << std::endl;
  std::cout << "points.cols():\n" << points.cols() << std::endl;

  std::cout << "point 0:\n" << points.col(0) << std::endl;

  std::cout << "point 1:\n" << points.col(1) << std::endl;

  // Define the downsampling factor (e.g., keep every nth point)
  int n = 5;

  std::cout << "points.cols():" << points.cols() << std::endl;

  // Calculate the number of points in the downsampled cloud
  int downsampledSize = float(points.cols()) / float(n);
  std::cout << "downsampledSize:" << downsampledSize << std::endl;

  // Create a matrix to hold the downsampled points
  Eigen::Matrix<float, 3, Eigen::Dynamic> downsampledPoints(3, downsampledSize);

  // Copy every nth point
  for (int i = 0, j = 0; i < points.cols(); i += n, ++j) {
    std::cout << "i:" << i << std::endl;

    downsampledPoints.col(j) = points.col(i);
  }

  // Now downsampledPoints contains the downsampled point cloud
  // ... Further processing ...

  std::cout << "Downsampled Points Matrix:\n" << downsampledPoints << std::endl;

  // Open a file in write mode.
  std::ofstream outFile("downsampled_points.csv");

  if (outFile.is_open()) {
    // Write each point to the file
    for (int i = 0; i < downsampledPoints.cols(); ++i) {
      outFile << downsampledPoints(0, i) << "," << downsampledPoints(1, i)
              << "," << downsampledPoints(2, i) << "\n";
    }

    outFile.close(); // Close the file after writing is done
    std::cout << "Downsampled point cloud written to downsampled_points.csv"
              << std::endl;
  } else {
    std::cerr << "Unable to open file for writing." << std::endl;
  }
}

void quaterniondFullPoseMultipication() {
  // Replace these values with your actual pose values
  double x1 = 1.0, y1 = 0.0, z1 = 0.0;
  double q_w1 = 1.0, q_x1 = 0.0, q_y1 = 0.0, q_z1 = 0.0;
  double x2 = 1.0, y2 = 0.0, z2 = 0.0;
  double q_w2 = 1.0, q_x2 = 0.0, q_y2 = 0.0, q_z2 = 0.0;

  Eigen::Affine3d pose1 = Eigen::Translation3d(x1, y1, z1) *
                          Eigen::Quaterniond(q_w1, q_x1, q_y1, q_z1);
  Eigen::Affine3d pose2 = Eigen::Translation3d(x2, y2, z2) *
                          Eigen::Quaterniond(q_w2, q_x2, q_y2, q_z2);

  Eigen::Affine3d result = pose1 * pose2;

  Eigen::Vector3d res_translation = result.translation();
  Eigen::Quaterniond res_quaternion(result.rotation());

  std::cout << "Resulting Pose Translation: " << res_translation.transpose()
            << std::endl;
  std::cout << "Resulting Pose Quaternion: " << res_quaternion.w() << " "
            << res_quaternion.x() << " " << res_quaternion.y() << " "
            << res_quaternion.z() << std::endl;
}

/**/
class ROSOdometryType {};
class ROSOrientationType {};
class ROSPointCloudType {};

class MavlinkOdometryType {};
class MavlinkOrientationType {};
class MavlinkPointCloudType {};

////////////////////////////////// templated classes + virtual function
/////////////////////////////////////
namespace impl1 {
// Base interface class
class SensorDataInterface {
public:
  // Template method for adding Odometry
  template <typename OdometryType>
  void addOdometryData(const OdometryType &odometryData) {}

  // Template method for adding Orientation or gimbal
  template <typename OrientationType>
  void addOrientationData(const OrientationType &orientationData) {}

  // Template method for adding PointCloud
  template <typename PointCloudType>
  void addPointCloudData(const PointCloudType &pointCloudData) {}

  virtual void processOdometry() = 0;
  virtual void processOrientation() = 0;
  virtual void processPointCloud() = 0;
};

////////////////////////////////// Derived class for ROS messages
/////////////////////////////////////
class ROSDataInterface : public SensorDataInterface {
public:
  // Using base class template methods as-is
  using SensorDataInterface::addOdometryData;
  using SensorDataInterface::addOrientationData;
  using SensorDataInterface::addPointCloudData;

  // ... ROS-specific methods ...
  void processOdometry() override {}
  void processOrientation() override {}
  void processPointCloud() override {}
};

class MAVLinkRangeDataInterface : public SensorDataInterface {
public:
  void addOdometryData(const MavlinkOdometryType &odometryData);
  void addOrientationData(const MavlinkOrientationType &orientationData);
  void addPointCloudData(const MavlinkPointCloudType &pointCloudData);
  // ... MAVLink and Range Image specific methods ...
  void processOdometry() override {}
  void processOrientation() override {}
  virtual void processPointCloud() override {}
};
} // namespace impl1

////////////////////////////////// Type-Erased Wrapper Classes
/////////////////////////////////////
namespace impl2 {
// Abstract base class for Odometry data wrapper
class OdometryWrapper {
public:
  virtual ~OdometryWrapper() {}
  // ... Common interface methods ...
};

// Templated subclass for Odometry data of any type
template <typename OdometryType>
class OdometryWrapperImpl : public OdometryWrapper {
  OdometryType data;

public:
  OdometryWrapperImpl(const OdometryType &odometryData) : data(odometryData) {}
};

class OrientationWrapper {
public:
  virtual ~OrientationWrapper() {}
  // ... Common interface methods ...
};

// Templated subclass for Orientation data of any type
template <typename OrientationType>
class OrientationWrapperImpl : public OrientationWrapper {
  OrientationType data;

public:
  OrientationWrapperImpl(const OrientationType &OrientationData)
      : data(OrientationData) {}
};

class PointCloudWrapper {
public:
  virtual ~PointCloudWrapper() {}
  // ... Common interface methods ...
};

// Templated subclass for PointCloud data of any type
template <typename PointCloudType>
class PointCloudWrapperImpl : public PointCloudWrapper {
  PointCloudType data;

public:
  PointCloudWrapperImpl(const PointCloudType &PointCloudData)
      : data(PointCloudData) {}
};

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

} // namespace impl2

////////////////////////////////// Overloaded Virtual Functions
/////////////////////////////////////

namespace impl3 {
class SensorDataInterface {
public:
  virtual void addOdometryData(const ROSOdometryType &odometryData) {}
  virtual void addOdometryData(const MavlinkOdometryType &odometryData) {}

  virtual void addOrientationData(const ROSOrientationType &orientationData) {}
  virtual void
  addOrientationData(const MavlinkOrientationType &orientationData) {}

  virtual void addPointCloudData(const ROSPointCloudType &pointCloudData) {}
  virtual void addPointCloudData(const MavlinkPointCloudType &pointCloudData) {}

  virtual void processOdometry() = 0;
  virtual void processOrientation() = 0;
  virtual void processPointCloud() = 0;
};

} // namespace impl3

struct robot {

  // std::function solver;
  // std::bind()
  // std::invoke()
  // std::placeholders
  // std::vector<double> (*solver_fn_ptr)(double start, double goal);

  std::function<std::vector<double>(double, double)> solver_fn_ptr;

  void exe(double start, double goal) {
    std::vector<double> traj = solver_fn_ptr(start, goal);
    for (auto t : traj) {
      std::cout << t << std::endl;
    }
  }
};

std::vector<double> solver1(double start, double goal) {
  std::vector<double> traj = {start, (start + goal) / 2, goal};
  return traj;
}

std::vector<double> solver2(double start, double goal) {
  std::vector<double> traj = {start, (start + goal) / 4, (start + goal) * 3 / 4,
                              goal};
  return traj;
}

int circularBufferSimpleExample() {

  std::size_t bufferSize = 10;

  CircularBuffer<int> buffer(bufferSize);

  for (int i = 0; i < bufferSize; ++i) {
    buffer.push(i);
  }

  buffer.push(bufferSize +
              1); // Overwrites the oldest data when the buffer is full
  buffer.push(bufferSize + 2);
  buffer.push(bufferSize + 3);

  while (!buffer.empty()) {
    std::cout << buffer.pop() << std::endl;
  }

  return 0;
}

int circularBufferEigenExample() {

  std::size_t bufferSize = 3;

  CircularBuffer<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> buffer(
      bufferSize);

  // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> downsampledPoints;
  // Eigen::MatrixXf m;
  // m.resize(2, 2);

  const int rows = 2;
  const int cols = 3;

  for (int i = 0; i < bufferSize; ++i) {
    // Create a matrix and initialize it with random values
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Random(rows, cols);
    buffer.push(matrix);
  }

  buffer.push(Eigen::MatrixXf::Random(
      rows, cols)); // Overwrites the oldest data when the buffer is full
  buffer.push(Eigen::MatrixXf::Random(rows, cols));

  while (!buffer.empty()) {
    std::cout << buffer.pop() << "\n----" << std::endl;
  }

  return 0;
}

int main(int argc, char **argv) {

  circularBufferEigenExample();

  robot r1;

  r1.solver_fn_ptr =
      std::bind(solver2, std::placeholders::_2, std::placeholders::_1);
  r1.exe(1, 10);

  //////////////////////////////// impl1 /////////////////////////////////
  {
    // Create an instance of a derived class
    impl1::SensorDataInterface *sensorInterface =
        new impl1::ROSDataInterface(); // or MAVLinkRangeDataInterface

    ROSOdometryType rosOdometryData;
    ROSOrientationType rosOrientationData;
    ROSPointCloudType rosPointCloudData;

    // Adding data (these are not virtual, just templated methods)
    sensorInterface->addOdometryData(rosOdometryData);
    sensorInterface->addOrientationData(rosOrientationData);
    sensorInterface->addPointCloudData(rosPointCloudData);

    // Processing data (these are virtual, overridden in derived classes)
    sensorInterface->processOdometry();
    sensorInterface->processOrientation();
    sensorInterface->processPointCloud();
  }

  //////////////////////////////// impl2 Type-Erased Wrapper Classes
  /////////////////////////////////////
  {

    // ros data
    ROSPointCloudType rosPointCloud;
    auto rosPointcloudData =
        std::make_shared<impl2::PointCloudWrapperImpl<ROSPointCloudType>>(
            rosPointCloud);
    impl2::SensorDataInterface *sensorInterfaceROS =
        new impl2::ROSDataInterface();
    sensorInterfaceROS->addPointCloudData(rosPointcloudData);
    sensorInterfaceROS->processPointCloud();

    // mavlink data

    MavlinkPointCloudType mavlinkPointCloud;
    auto mavLinkPointcloudData =
        std::make_shared<impl2::PointCloudWrapperImpl<MavlinkPointCloudType>>(
            mavlinkPointCloud);

    impl2::SensorDataInterface *sensorInterfaceMavlink =
        new impl2::MavlinkDataInterface();
    sensorInterfaceMavlink->addPointCloudData(mavLinkPointcloudData);
    sensorInterfaceMavlink->processPointCloud();
  }

  //////////////////////////////// impl3: Overloaded Virtual Functions
  ////////////////////////////////////
  {
    // // Creating an instance of OdometryWrapper for a specific data type
    // auto odometryData =
    // std::make_shared<impl3::OdometryWrapperImpl<ROSOdometryType>>(rosOdometry);

    // // Using it with SensorDataInterface
    // impl3::SensorDataInterface* sensorInterface = new
    // impl3::ROSDataInterface();
    // sensorInterface->addOdometryData(odometryData);
  }

  // Eigen::Matrix4f transformationMatrix;
  // transformationMatrix.setIdentity();  // Set to identity matrix
  // std::cout << transformationMatrix << std::endl;

  return 0;
}
