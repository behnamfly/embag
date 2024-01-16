#pragma once
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
