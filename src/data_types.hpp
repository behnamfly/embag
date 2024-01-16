#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

struct Point {
  float x;
  float y;
  float z;
  Point(float posX = 0, float posY = 0, float posZ = 0)
      : x(posX), y(posY), z(posZ) {}
};

// data structure representing a frame
struct Pose {
  float positionX;
  float positionY;
  float positionZ;
  float orientationX;
  float orientationY;
  float orientationZ;
  float orientationW;
  Pose(float posX = 0, float posY = 0, float posZ = 0, float oriX = 0,
       float oriY = 0, float oriZ = 0, float oriW = 1)
      : positionX(posX), positionY(posY), positionZ(posZ), orientationX(oriX),
        orientationY(oriY), orientationZ(oriZ), orientationW(oriW) {}
};

// data structure representing trajectory
struct TimeStampedPose {
  long long int timestampSec;      // Timestamp in seconds
  long long int timestampNanoSec;  // Timestamp in nanoseconds
  long long int combinedTimestamp; // Combined timestamp in nanoseconds

  Pose pose;

  void print() const {
    std::cout << "Timestamp: " << combinedTimestamp << " ns (" << timestampSec
              << " s + " << timestampNanoSec << " ns), Pose: "
              << "Position(" << pose.positionX << ", " << pose.positionY << ", "
              << pose.positionZ << "), "
              << "Orientation(" << pose.orientationX << ", "
              << pose.orientationY << ", " << pose.orientationZ << ", "
              << pose.orientationW << ")" << std::endl;
  }
};

struct TimeStampedPoints {
  long long int timestampSec;
  long long int timestampNanoSec;
  long long int combinedTimestamp;

  Eigen::Matrix<float, 3, Eigen::Dynamic> pointsMatrix;

  void print() const {
    std::cout << "Timestamp: " << combinedTimestamp
              << " ns, Points:" << std::endl;
    for (int i = 0; i < pointsMatrix.cols(); ++i) {
      std::cout << pointsMatrix.col(i) << "\n---" << std::endl;
    }
  }
};

#endif // DATA_TYPES_HPP
