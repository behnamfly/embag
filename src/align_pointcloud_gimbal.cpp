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
#include "pointcloudaligner.hpp"

/*

Tof in gimbal
gimbal in body frame
boday in local frame


 # XYZW
gimbal_in_body=Pose(
     r=(0.188, 0.03285, 0.0019),
     q=(0, 0, 0, 1),  # Dynamic
 ),
tof_in_gimbal=Pose(
 r=(0.008997, -0.0234, -0.016215),
 q=(0.7071068, 0, 0.7071068, 0),  # XYZW
),





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

// struct Point {
//     float x;
//     float y;
//     float z;
// };

// // data structure representing a frame
// struct Pose {
//     float positionX;
//     float positionY;
//     float positionZ;
//     float orientationX;
//     float orientationY;
//     float orientationZ;
//     float orientationW;
// };

// // data structure representing trajectory
// struct TimeStampedPose {
//     qint64 timestampSec;       // Timestamp in seconds
//     qint64 timestampNanoSec;   // Timestamp in nanoseconds
//     qint64 combinedTimestamp;  // Combined timestamp in nanoseconds

//     Pose pose;

//     void print() const
//     {
//         std::cout << "Timestamp: " << combinedTimestamp << " ns (" << timestampSec << " s + " << timestampNanoSec
//                   << " ns), Pose: "
//                   << "Position(" << pose.positionX << ", " << pose.positionY << ", " << pose.positionZ << "), "
//                   << "Orientation(" << pose.orientationX << ", " << pose.orientationY << ", " << pose.orientationZ
//                   << ", " << pose.orientationW << ")" << std::endl;
//     }
// };

// // data structure representing timestamped pointcloud
// struct TimeStampedPoints {
//     qint64 timestampSec;
//     qint64 timestampNanoSec;
//     qint64 combinedTimestamp;
//     QVector<Point> points;

//     void print() const
//     {
//         std::cout << "Timestamp: " << combinedTimestamp << " ns, Points:" << std::endl;
//         for (const Point &point : points) {
//             std::cout << "  (x: " << point.x << ", y: " << point.y << ", z: " << point.z << ")" << std::endl;
//         }
//     }
// };

void processData(const QJsonDocument &document, QVector<TimeStampedPose> &timeStampedPoses)
{
    QJsonObject rootObject = document.object();

    QJsonObject secObject = rootObject["header.stamp.sec"].toObject();

    std::cout << "secObject.size(): " << secObject.size() << std::endl;

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

void processData(const QJsonDocument &document, QVector<TimeStampedPoints> &timeStampedPointsVec)
{
    QJsonObject rootObject = document.object();

    QJsonObject secObject = rootObject["header.stamp.sec"].toObject();

    // Reserve space in timeStampedPoses based on the size of secObject
    timeStampedPointsVec.reserve(secObject.size());

    QJsonObject nanosecObject = rootObject["header.stamp.nanosec"].toObject();
    QJsonObject pointsObject = rootObject["points"].toObject();

    for (auto it = secObject.constBegin(); it != secObject.constEnd(); ++it) {
        const QString &key = it.key();
        qDebug() << "key: " << key;

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

        // std::cout << "pointKeys.size(): " << pointKeys.size() << std::endl;

        timeStampedPoints.pointsMatrix.resize(3, pointKeys.size());

        int i = 0;
        for (const QString &pointKey : pointKeys) {

            Eigen::Vector3f col(
                xObject.value(pointKey).toDouble(), yObject.value(pointKey).toDouble(),
                zObject.value(pointKey).toDouble()
            );

            timeStampedPoints.pointsMatrix.col(i) = col;
            i++;
        }
        timeStampedPointsVec.push_back(timeStampedPoints);
    }
}

void printTimeStampedPoints(const TimeStampedPoints &timeStampedPoints)
{
    std::cout << "Timestamp: " << timeStampedPoints.combinedTimestamp << " ns, Points:" << std::endl;
    // for (const Point &point : timeStampedPoints.points) {
    //     std::cout << "  (x: " << point.x << ", y: " << point.y << ", z: " << point.z << ")" << std::endl;
    // }

    for (std::size_t i = 0; i < timeStampedPoints.pointsMatrix.cols(); i++) {
        std::cout << "timeStampedPoints: " << timeStampedPoints.pointsMatrix.col(i) << std::endl;
    }
}

void readAndParseJson1(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open" << fileName;
        return;
    }

    QByteArray jsonData = file.readAll();
    file.close();

    QJsonDocument document = QJsonDocument::fromJson(jsonData);
    if (document.isNull()) {
        qDebug() << "Failed to create JSON doc.";
        return;
    }
    if (!document.isObject()) {
        qDebug() << "JSON is not an object.";
        return;
    }

    QJsonObject jsonObject = document.object();

    QJsonObject::iterator it;
    for (it = jsonObject.begin(); it != jsonObject.end(); ++it) {
        qDebug() << "Key:" << it.key();
        QJsonObject innerObject = it.value().toObject();

        QJsonObject::iterator innerIt;
        for (innerIt = innerObject.begin(); innerIt != innerObject.end(); ++innerIt) {
            qDebug() << "  Timestamp:" << innerIt.key() << "Value:" << innerIt.value().toDouble();
        }
    }
}

QVector<TimeStampedPose> readAndParseJson(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open" << fileName;
        return QVector<TimeStampedPose>();
    }

    QByteArray jsonData = file.readAll();
    file.close();

    QJsonDocument document = QJsonDocument::fromJson(jsonData);
    if (document.isNull()) {
        qDebug() << "Failed to create JSON doc.";
        return QVector<TimeStampedPose>();
    }
    if (!document.isObject()) {
        qDebug() << "JSON is not an object.";
        return QVector<TimeStampedPose>();
    }

    QJsonObject jsonObject = document.object();
    QMap<qint64, Pose> posesMap;

    for (const QString &key : jsonObject.keys()) {
        QJsonObject orientationObject = jsonObject[key].toObject();

        for (auto it = orientationObject.begin(); it != orientationObject.end(); ++it) {
            qint64 timestamp = it.key().toLongLong();
            float value = static_cast<float>(it.value().toDouble());

            if (!posesMap.contains(timestamp)) {
                posesMap[timestamp] = Pose();
            }

            if (key == "x")
                posesMap[timestamp].orientationX = value;
            else if (key == "y")
                posesMap[timestamp].orientationY = value;
            else if (key == "z")
                posesMap[timestamp].orientationZ = value;
            else if (key == "w")
                posesMap[timestamp].orientationW = value;
        }
    }

    QVector<TimeStampedPose> timeStampedPoses;
    for (auto it = posesMap.begin(); it != posesMap.end(); ++it) {
        TimeStampedPose timeStampedPose;

        timeStampedPose.timestampNanoSec = 0;
        timeStampedPose.timestampSec = it.key();

        // timeStampedPose.combinedTimestamp = 1000000LL * timeStampedPose.timestampSec +
        //                                     timeStampedPose.timestampNanoSec;

        qint64 combinedTimestamp = 1000000LL * it.key();
        timeStampedPose.combinedTimestamp = combinedTimestamp;

        timeStampedPose.pose = it.value();
        timeStampedPoses.append(timeStampedPose);
    }

    // for (const TimeStampedPose &pose : timeStampedPoses) {
    //     qDebug() << "Timestamp:" << pose.combinedTimestamp << " timeStampedPose.timestampSec: " << pose.timestampSec
    //              << "Orientation X:" << pose.pose.orientationX << "Orientation Y:" << pose.pose.orientationY
    //              << "Orientation Z:" << pose.pose.orientationZ << "Orientation W:" << pose.pose.orientationW;
    // }

    return timeStampedPoses;
}

int main(int argc, char **argv)
{

    std::string basePath =
        "C:/Users/basadi/.conda/envs/VirtualCam/src/data/asio_lidar/gimbal/"
        "session_2023-12-13-13-35-23_0_sliced_extracted/";

    //
    // session_2023-12-13-13-35-23_0_sliced_extracted
    // session_2023-12-13-13-39-50_2_extracted

    // processing mount control orientation gimbal
    std::string mount_control_orientation;
    mount_control_orientation = basePath + "mavros_mount_control_orientation.json";
    std::cout << "mount_control_orientation: " << mount_control_orientation << std::endl;

    QVector<TimeStampedPose> gimbalPosesTimeStamped = readAndParseJson(QString::fromStdString(mount_control_orientation)
    );

    std::string mavros_odometry_in;
    mavros_odometry_in = basePath + "mavros_odometry_in.json";

    std::cout << "mavros_odometry_in: " << mavros_odometry_in << std::endl;

    QFile file_mavros_odometry_in(mavros_odometry_in.c_str());
    if (!file_mavros_odometry_in.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return -1;
    }
    QString jsonText = file_mavros_odometry_in.readAll();
    file_mavros_odometry_in.close();

    QJsonDocument document = QJsonDocument::fromJson(jsonText.toUtf8());
    QVector<TimeStampedPose> odomInWorldFrame;
    processData(document, odomInWorldFrame);

    std::cout << "odom:" << std::endl;
    for (const auto &d : odomInWorldFrame) {
        d.print();
    }

    // processing lidar data
    std::string royale_camera_driver_point_cloud;
    royale_camera_driver_point_cloud = basePath + "royale_camera_driver_point_cloud.json";

    std::cout << "royale_camera_driver_point_cloud: " << royale_camera_driver_point_cloud << std::endl;

    QFile file_royale_camera_driver_point_cloud(royale_camera_driver_point_cloud.c_str());
    if (!file_royale_camera_driver_point_cloud.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return -1;
    }

    const qint64 chunkSize = 1024 * 1024;  // 1 MB

    QByteArray fileData;
    // int i = 0;
    while (!file_royale_camera_driver_point_cloud.atEnd()) {
        QByteArray chunk = file_royale_camera_driver_point_cloud.read(chunkSize);
        fileData.append(chunk);
        // std::cout << i++ << std::endl;
    }

    // std::cout << "-----------" << std::endl;

    file_royale_camera_driver_point_cloud.close();
    QJsonDocument document_royale_camera_driver_point_cloud = QJsonDocument::fromJson(fileData);

    QVector<TimeStampedPoints> lidarPointCloud;
    processData(document_royale_camera_driver_point_cloud, lidarPointCloud);

    // for (const auto &l : lidarPointCloud) {
    //     l.print();
    // }

    std::cout << "gimbalPosesTimeStamped.size(): " << gimbalPosesTimeStamped.size() << std::endl;

    std::cout << "odomInWorldFrame.size(): " << odomInWorldFrame.size() << std::endl;

    std::cout << "lidarPointCloud.size(): " << lidarPointCloud.size() << std::endl;

    long long int delay_mili_second = 0LL;
    long long int delay_nano_second = delay_mili_second * 1000000LL;

    vtkNew<vtkPoints> vtkpointcloudInWorld;
    vtkNew<vtkPolyData> polyData;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSum(new pcl::PointCloud<pcl::PointXYZ>());

    /*        std::cout << "*******************************************************" << std::endl;
     */

    /*

    Tof in gimbal
    gimbal in body frame
    boday in local frame
*/

    /*
        tof_in_gimbal=Pose(
         r=(0.008997, -0.0234, -0.016215),
         q=(0.7071068, 0, 0.7071068, 0),  # XYZW
        ),
    */

    float x, y, z, w, xi, yj, zk;

    x = 0.008997;
    y = -0.0234;
    z = -0.016215;

    xi = 0.7071068;
    yj = 0;
    zk = 0.7071068;
    w = 0;

    TimeStampedPose TofInGimbalFrame;

    TofInGimbalFrame.pose.orientationW = w;
    TofInGimbalFrame.pose.orientationX = xi;
    TofInGimbalFrame.pose.orientationY = yj;
    TofInGimbalFrame.pose.orientationZ = zk;

    TofInGimbalFrame.pose.positionX = x;
    TofInGimbalFrame.pose.positionY = y;
    TofInGimbalFrame.pose.positionZ = z;

    for (std::size_t pointcloudIndex = 0; pointcloudIndex < lidarPointCloud.size(); pointcloudIndex++) {
        std::cout << "pointcloudIndex : " << pointcloudIndex;

        int correspondingTrajectoryIndex = 0;
        for (int trajectoryIndex = correspondingTrajectoryIndex; trajectoryIndex < odomInWorldFrame.size();
             trajectoryIndex++) {
            if (lidarPointCloud[pointcloudIndex].combinedTimestamp - delay_nano_second <
                odomInWorldFrame[trajectoryIndex].combinedTimestamp) {
                // poseOfDroneInWOrld = pose.pose;
                correspondingTrajectoryIndex = trajectoryIndex - 1;
                if (correspondingTrajectoryIndex > 0) {
                    std::cout << " trajectoryIndex: " << correspondingTrajectoryIndex;

                    // std::cout << "lidar.timestampSec: " << lidarPointCloud[pointcloudIndex].timestampSec
                    //           << " drone.timestampSec: " << odomInWorldFrame[trajectoryIndex].timestampSec <<
                    //           std::endl;
                }
                break;
            }
        }

        if (correspondingTrajectoryIndex <= 0) {
            continue;
        }

        int correspondingGimbalIndex = 0;

        // std::cout << "lidarPointCloud[" << pointcloudIndex
        //           << "].combinedTimestamp: " << lidarPointCloud[pointcloudIndex].combinedTimestamp << std::endl;

        for (std::size_t gimbalIndex = correspondingGimbalIndex; gimbalPosesTimeStamped.size(); gimbalIndex++) {

            // std::cout << "gimbalPosesTimeStamped[" << gimbalIndex
            //           << "].combinedTimestamp: " << gimbalPosesTimeStamped[gimbalIndex].combinedTimestamp <<
            //           std::endl;

            if (lidarPointCloud[pointcloudIndex].combinedTimestamp <
                gimbalPosesTimeStamped[gimbalIndex].combinedTimestamp) {

                if (gimbalIndex > 0) {

                    correspondingGimbalIndex = gimbalIndex;

                    std::cout << " correspondingGimbalIndex: " << correspondingGimbalIndex << std::endl;
                }
                break;
            }
        }

        PointCloudAligner aligner;
        aligner.setNumberOfPointTodecimate(50);

        aligner.setMaxDistanceClamping(4);
        aligner.setMinDistanceClamping(0.4);

        // setting pointcloud
        aligner.setPointcloud(lidarPointCloud[pointcloudIndex]);

        /*
              # XYZW
             gimbal_in_body=Pose(
                  r=(0.188, 0.03285, 0.0019),
                  q=(0, 0, 0, 1),  # Dynamic
              ),

        */

        /**/
        gimbalPosesTimeStamped[correspondingGimbalIndex].pose.positionX = 0.188;
        gimbalPosesTimeStamped[correspondingGimbalIndex].pose.positionY = 0.03285;
        gimbalPosesTimeStamped[correspondingGimbalIndex].pose.positionZ = 0.0019;

        gimbalPosesTimeStamped[correspondingGimbalIndex]
            .pose.orientationW = +gimbalPosesTimeStamped[correspondingGimbalIndex].pose.orientationW;
        gimbalPosesTimeStamped[correspondingGimbalIndex]
            .pose.orientationX = -gimbalPosesTimeStamped[correspondingGimbalIndex].pose.orientationX;
        gimbalPosesTimeStamped[correspondingGimbalIndex]
            .pose.orientationY = -gimbalPosesTimeStamped[correspondingGimbalIndex].pose.orientationY;
        gimbalPosesTimeStamped[correspondingGimbalIndex]
            .pose.orientationZ = -gimbalPosesTimeStamped[correspondingGimbalIndex].pose.orientationZ;

        aligner.setGimbalInBodyFrame(gimbalPosesTimeStamped[correspondingGimbalIndex]);
        aligner.setOdomInWorldFrame(odomInWorldFrame[correspondingTrajectoryIndex]);
        aligner.setTofInGimbalFrame(TofInGimbalFrame);

        // aligner.decimation();
        // aligner.clamping();

        // std::cout << "pointcloudInLidarDecimatedClamped: " << aligner.getDecimatedPoints().pointsMatrix;

        TimeStampedPoints pointcloudInWorldDownSampled = aligner.getPointCloudInWorldCoordinate();
        // std::cout << "pointcloudInWorldDownSampled.pointsMatrix.cols(): " <<
        // pointcloudInWorldDownSampled.pointsMatrix
        //           << std::endl;

        for (std::size_t i = 0; i < aligner.getPointCloudInWorldCoordinate().pointsMatrix.cols(); i++) {
            vtkpointcloudInWorld->InsertNextPoint(
                aligner.getPointCloudInWorldCoordinate().pointsMatrix.col(i).x(),
                aligner.getPointCloudInWorldCoordinate().pointsMatrix.col(i).y(),
                aligner.getPointCloudInWorldCoordinate().pointsMatrix.col(i).z()
            );
        }
    }

    polyData->SetPoints(vtkpointcloudInWorld);

    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    writer->SetFileName("map.ply");
    writer->SetInputData(polyData);  // Assuming 'polyData' is your vtkPolyData object
    writer->SetFileTypeToBinary();
    writer->Write();

    // std::string file_name = "cloud_filtered.ply";
    // bool binary = true;
    // bool use_camera = false;
    // pcl::PLYWriter plyWriter;

    // vtkNew<vtkPolyData> bar;

    // // plyWriter.write(file_name, *cloudSum, binary, use_camera);

    // pcl::io::pointCloudTovtkPolyData(*cloudSum, bar);
    // writer->SetFileName("map_filtered.ply");
    // writer->SetInputData(bar);  // Assuming 'polyData' is your vtkPolyData object
    // writer->SetFileTypeToBinary();
    // writer->Write();
}
