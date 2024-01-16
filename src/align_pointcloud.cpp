#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkOctreePointLocator.h>
#include <vtkPLYWriter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <Eigen/Geometry>
#include <QFile>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include "ArgumentsParser.hpp"

/*
Orientation of Tof in Body
0   0   1
0  -1   0
1   0   0

Translation of Tof in Body
x  197 mm
y  9.45 mm
z  -14.3


To generate json file from ROS bags:
checkout and build asio-autonomy
then run

rosbag2json.exe <bagfile> -t <topic1>  -t <topic2>

for example:
rosbag2json.exe .\src\data\asio_lidar\session_2023-03-02-13-00-24_0.bag -t /mavros/odometry/in  -t
/royale_camera_driver/point_cloud
*/
struct Point {
    float x;
    float y;
    float z;
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
};

// data structure representing trajectory
struct TimeStampedPose {
    qint64 timestampSec;       // Timestamp in seconds
    qint64 timestampNanoSec;   // Timestamp in nanoseconds
    qint64 combinedTimestamp;  // Combined timestamp in nanoseconds

    Pose pose;

    void print() const
    {
        std::cout << "Timestamp: " << combinedTimestamp << " ns (" << timestampSec << " s + " << timestampNanoSec
                  << " ns), Pose: "
                  << "Position(" << pose.positionX << ", " << pose.positionY << ", " << pose.positionZ << "), "
                  << "Orientation(" << pose.orientationX << ", " << pose.orientationY << ", " << pose.orientationZ
                  << ", " << pose.orientationW << ")" << std::endl;
    }
};

// data structure representing timestamped pointcloud
struct TimeStampedPoints {
    qint64 timestampSec;
    qint64 timestampNanoSec;
    qint64 combinedTimestamp;
    QList<Point> points;

    void print() const
    {
        std::cout << "Timestamp: " << combinedTimestamp << " ns, Points:" << std::endl;
        for (const Point &point : points) {
            std::cout << "  (x: " << point.x << ", y: " << point.y << ", z: " << point.z << ")" << std::endl;
        }
    }
};

void processData(const QJsonDocument &document, QVector<TimeStampedPose> &timeStampedPoses)
{
    QJsonObject rootObject = document.object();

    QJsonObject secObject = rootObject["header.stamp.sec"].toObject();

    // Reserve space in timeStampedPoses based on the size of secObject
    timeStampedPoses.reserve(secObject.size());
    QJsonObject nanosecObject = rootObject["header.stamp.nanosec"].toObject();

    QJsonObject posXObject = rootObject["pose.pose.position.x"].toObject();
    QJsonObject posYObject = rootObject["pose.pose.position.y"].toObject();
    QJsonObject posZObject = rootObject["pose.pose.position.z"].toObject();

    QJsonObject orientationXObject = rootObject["pose.pose.orientation.x"].toObject();
    QJsonObject orientationYObject = rootObject["pose.pose.orientation.y"].toObject();
    QJsonObject orientationZObject = rootObject["pose.pose.orientation.z"].toObject();
    QJsonObject orientationWObject = rootObject["pose.pose.orientation.w"].toObject();

    // ... Similarly for positionY, positionZ, orientationX, orientationY, orientationZ, orientationW

    for (auto it = secObject.constBegin(); it != secObject.constEnd(); ++it) {
        const QString &key = it.key();
        qint64 secPart = it.value().toVariant().toLongLong();
        qint64 nanosecPart = nanosecObject.value(key).toVariant().toLongLong();

        qint64 combinedTimestamp = secPart * 1000000000LL + nanosecPart;

        Pose pose;
        pose.positionX = posXObject.value(key).toDouble();
        pose.positionY = posYObject.value(key).toDouble();
        pose.positionZ = posZObject.value(key).toDouble();

        pose.orientationX = orientationXObject.value(key).toDouble();
        pose.orientationY = orientationYObject.value(key).toDouble();
        pose.orientationZ = orientationZObject.value(key).toDouble();
        pose.orientationW = orientationWObject.value(key).toDouble();

        TimeStampedPose timeStampedPose;
        timeStampedPose.timestampSec = secPart;
        timeStampedPose.timestampNanoSec = nanosecPart;
        timeStampedPose.combinedTimestamp = combinedTimestamp;
        timeStampedPose.pose = pose;

        timeStampedPoses.push_back(timeStampedPose);
    }
}

void processData(const QJsonDocument &document, QVector<TimeStampedPoints> &timeStampedPointsList)
{
    QJsonObject rootObject = document.object();

    QJsonObject secObject = rootObject["header.stamp.sec"].toObject();

    // Reserve space in timeStampedPoses based on the size of secObject
    timeStampedPointsList.reserve(secObject.size());

    QJsonObject nanosecObject = rootObject["header.stamp.nanosec"].toObject();
    QJsonObject pointsObject = rootObject["points"].toObject();

    for (auto it = secObject.constBegin(); it != secObject.constEnd(); ++it) {
        const QString &key = it.key();

        //        qDebug() << "key: " << key;

        qint64 secPart = it.value().toVariant().toLongLong();
        qint64 nanosecPart = nanosecObject.value(key).toVariant().toLongLong();

        qint64 combinedTimestamp = secPart * 1000000000LL + nanosecPart;

        QJsonObject pointSet = pointsObject.value(key).toObject();

        //        qDebug() << "pointSet: " << pointSet;

        QJsonObject xObject = pointSet["x"].toObject();
        QJsonObject yObject = pointSet["y"].toObject();
        QJsonObject zObject = pointSet["z"].toObject();

        TimeStampedPoints timeStampedPoints;
        timeStampedPoints.timestampSec = secPart;
        timeStampedPoints.timestampNanoSec = nanosecPart;
        timeStampedPoints.combinedTimestamp = combinedTimestamp;

        QStringList pointKeys = xObject.keys();

        for (const QString &pointKey : pointKeys) {
            Point point;
            point.x = xObject.value(pointKey).toDouble();
            point.y = yObject.value(pointKey).toDouble();
            point.z = zObject.value(pointKey).toDouble();
            timeStampedPoints.points.append(point);
        }
        timeStampedPointsList.push_back(timeStampedPoints);
    }
}

void printTimeStampedPoints(const TimeStampedPoints &timeStampedPoints)
{
    std::cout << "Timestamp: " << timeStampedPoints.combinedTimestamp << " ns, Points:" << std::endl;
    for (const Point &point : timeStampedPoints.points) {
        std::cout << "  (x: " << point.x << ", y: " << point.y << ", z: " << point.z << ")" << std::endl;
    }
}

int main(int argc, char **argv)
{

    // ArgumentsParser input(argc, argv);
    // if (input.argExists("-h") || input.argExists("--help") || argc == 1) {
    //     std::cerr << "usage: align_pointcloud "
    //                  " -p <pointcloud.json> "
    //                  " -o <odometry.json> "
    //                  "\n Examples of run:"
    //                  "\n align_pointcloud.exe -o mavros_odometry_in.json  -p royale_camera_driver_point_cloud.json"
    //               << std::endl;
    //     return 1;
    // }

    // processing trajectory (odometry)
    std::string mavros_odometry_in;
    mavros_odometry_in =
        "C:/Users/basadi/.conda/envs/VirtualCam/src/data/asio_lidar/session_2023-03-02-13-00-24_0_extracted/"
        "mavros_odometry_in.json";
    std::cout << "mavros_odometry_in: " << mavros_odometry_in << std::endl;

    QFile file_mavros_odometry_in(mavros_odometry_in.c_str());
    if (!file_mavros_odometry_in.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return -1;
    }
    QString jsonText = file_mavros_odometry_in.readAll();
    file_mavros_odometry_in.close();

    QJsonDocument document = QJsonDocument::fromJson(jsonText.toUtf8());
    QVector<TimeStampedPose> droneTrajectory;
    processData(document, droneTrajectory);

    // processing lidar data
    std::string royale_camera_driver_point_cloud;
    royale_camera_driver_point_cloud =
        "C:/Users/basadi/.conda/envs/VirtualCam/src/data/asio_lidar/session_2023-03-02-13-00-24_0_extracted/"
        "royale_camera_driver_point_cloud.json";

    std::cout << "royale_camera_driver_point_cloud: " << royale_camera_driver_point_cloud << std::endl;

    QFile file_royale_camera_driver_point_cloud(royale_camera_driver_point_cloud.c_str());
    if (!file_royale_camera_driver_point_cloud.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return -1;
    }
    QTextStream foo(&file_royale_camera_driver_point_cloud);

    const qint64 chunkSize = 1024 * 1024;  // 1 MB
    QString jsonText_royale_camera_driver_point_cloud;
    QByteArray fileData;
    while (!file_royale_camera_driver_point_cloud.atEnd()) {
        QByteArray chunk = file_royale_camera_driver_point_cloud.read(chunkSize);
        fileData.append(chunk);
    }
    file_royale_camera_driver_point_cloud.close();
    QJsonDocument document_royale_camera_driver_point_cloud = QJsonDocument::fromJson(fileData);

    QVector<TimeStampedPoints> lidarPointCloud;
    processData(document_royale_camera_driver_point_cloud, lidarPointCloud);

    //
    long long int delay_mili_second = 0LL;
    long long int delay_nano_second = delay_mili_second * 1000000LL;

    vtkNew<vtkPoints> vtkpointcloudInWorld;
    vtkNew<vtkPolyData> polyData;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSum(new pcl::PointCloud<pcl::PointXYZ>());

    for (std::size_t pointcloudIndex = 0; pointcloudIndex < lidarPointCloud.size(); pointcloudIndex++) {

        int correspondingTrajectoryIndex = 0;
        for (int trajectoryIndex = 0; trajectoryIndex < droneTrajectory.size(); trajectoryIndex++) {
            if (lidarPointCloud[pointcloudIndex].combinedTimestamp - delay_nano_second <
                droneTrajectory[trajectoryIndex].combinedTimestamp) {
                // poseOfDroneInWOrld = pose.pose;
                correspondingTrajectoryIndex = trajectoryIndex - 1;
                if (correspondingTrajectoryIndex > 0) {
                    std::cout << "pointcloudIndex : " << pointcloudIndex
                              << " trajectoryIndex: " << correspondingTrajectoryIndex << std::endl;
                }
                break;
            }
        }

        if (correspondingTrajectoryIndex <= 0) {
            continue;
        }

        // Frame parameters
        float x = droneTrajectory[correspondingTrajectoryIndex].pose.positionX; /* frame translation x */
        float y = droneTrajectory[correspondingTrajectoryIndex].pose.positionY; /* frame translation y */
        float z = droneTrajectory[correspondingTrajectoryIndex].pose.positionZ; /* frame translation z */

        float w = droneTrajectory[correspondingTrajectoryIndex].pose.orientationW; /* quaternion w */
        float xi = droneTrajectory[correspondingTrajectoryIndex].pose.orientationX /* quaternion x */;
        float yj = droneTrajectory[correspondingTrajectoryIndex].pose.orientationY /* quaternion y */;
        float zk = droneTrajectory[correspondingTrajectoryIndex].pose.orientationZ /* quaternion z */;

        // Define the frame
        Eigen::Translation3f translation(x, y, z);
        Eigen::Quaternionf orientation(w, xi, yj, zk);  // Ensure the quaternion is normalized

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

        cloud->points.clear();
        cloud_filtered->points.clear();

        for (const auto &pointcloudPoint : lidarPointCloud[pointcloudIndex].points) {
            // Point coordinates
            float px = pointcloudPoint.x; /* point x */
            float py = pointcloudPoint.y; /* point y */
            float pz = pointcloudPoint.z /* point z */;

            // Define the point
            Eigen::Vector3f point_in_lidar_frame(px, py, pz);

            // Define the transformation matrix as an Eigen::Affine3d
            Eigen::Affine3f transformFromLidarToBody = Eigen::Affine3f::Identity();
            transformFromLidarToBody.matrix() << 0, 0, 1, 197 / 1000, 0, -1, 0, 9.45 / 1000, 1, 0, 0, -14.3 / 1000, 0,
                0, 0, 1;

            Eigen::Vector3f pointInBodyFrame = transformFromLidarToBody * point_in_lidar_frame.homogeneous();

            // std::cout << "point_in_lidar_frame:\n" << point_in_lidar_frame << std::endl;

            // Compose the transformation
            Eigen::Affine3f transformFromBodyToWorld = translation * orientation;

            // Apply the transformation
            Eigen::Vector3f pointInWorld = transformFromBodyToWorld * pointInBodyFrame;

            // std::cout << "pointInWorld:\n" << pointInWorld<< std::endl;

            vtkpointcloudInWorld->InsertNextPoint(pointInWorld.x(), pointInWorld.y(), pointInWorld.z());
            cloud->points.push_back(pcl::PointXYZ{pointInWorld.x(), pointInWorld.y(), pointInWorld.z()});
            //  cloud->points.push_back(pcl::PointXYZ{1, 1, 1});
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor;

        sor.setInputCloud(cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*cloud_filtered);
        *cloudSum += *cloud_filtered;
    }

    polyData->SetPoints(vtkpointcloudInWorld);

    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    writer->SetFileName("map.ply");
    writer->SetInputData(polyData);  // Assuming 'polyData' is your vtkPolyData object
    writer->SetFileTypeToBinary();
    writer->Write();

    std::string file_name = "cloud_filtered.ply";
    bool binary = true;
    bool use_camera = false;
    pcl::PLYWriter plyWriter;

    vtkNew<vtkPolyData> bar;

    // plyWriter.write(file_name, *cloudSum, binary, use_camera);

    pcl::io::pointCloudTovtkPolyData(*cloudSum, bar);
    writer->SetFileName("map_filtered.ply");
    writer->SetInputData(bar);  // Assuming 'polyData' is your vtkPolyData object
    writer->SetFileTypeToBinary();
    writer->Write();
}
//
