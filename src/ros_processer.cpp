#include "ros_processer.hpp"
#include <vtkPLYWriter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <Eigen/Sparse>
#include <cmath>
#include <iomanip>

void ROSProcesser::processBagData(
    Embag::View &view, const std::string &mountControlOrientationTopic, const std::string &odometryTopic,
    const std::string &pointcloudTopic
)
{

    /*
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

    std::initializer_list<std::string> topics{mountControlOrientationTopic, odometryTopic, pointcloudTopic};

    vtkNew<vtkPoints> vtkpointcloudInWorld;
    vtkNew<vtkPolyData> polyData;
    int cloud_id = 0;

    for (const auto &message : view.getMessages(topics)) {

        /*
        std::cout << std::fixed << message->timestamp.to_sec() << " : " << message->topic << std::endl;

        std::cout << "time is " << message->timestamp.secs << "." << message->timestamp.nsecs << std::endl;
        */
        if (message->topic == mountControlOrientationTopic) {
            // mountControlOrientationTopicis of type: geometry_msgs/Quaternion
            double x, y, z, w;

            x = message->data()["x"]->as<double>();
            y = message->data()["y"]->as<double>();
            z = message->data()["z"]->as<double>();
            w = message->data()["w"]->as<double>();

            /*
                  std::cout << "mount control orientation x: " << x << " y: " << y
                            << " z: " << z << " w: " << w << std::endl;
            */
            m_mount_control.pose.positionX = 0.188;
            m_mount_control.pose.positionY = 0.03285;
            m_mount_control.pose.positionZ = 0.0019;

            m_mount_control.pose.orientationX = -x;
            m_mount_control.pose.orientationY = -y;
            m_mount_control.pose.orientationZ = -z;
            m_mount_control.pose.orientationW = w;

        } else if (message->topic == odometryTopic) {
            // odometryTopic is of type nav_msgs/Odometry

            /*
            std::cout << message->data()["child_frame_id"]->as<std::string>()
                      << std::endl;
            */

            double pose_x, pose_y, pose_z;
            double q_x, q_y, q_z, q_w;

            pose_x = message->data()["pose"]->get("pose")->get("position")->get("x")->as<double>();
            pose_y = message->data()["pose"]->get("pose")->get("position")->get("y")->as<double>();
            pose_z = message->data()["pose"]->get("pose")->get("position")->get("z")->as<double>();

            // orientation

            q_x = message->data()["pose"]->get("pose")->get("orientation")->get("x")->as<double>();

            q_y = message->data()["pose"]->get("pose")->get("orientation")->get("y")->as<double>();

            q_w = message->data()["pose"]->get("pose")->get("orientation")->get("w")->as<double>();

            q_z = message->data()["pose"]->get("pose")->get("orientation")->get("z")->as<double>();

            if (abs(pose_x) > 10 || abs(pose_y) > 10 || abs(pose_z) > 10 || abs(q_x) > 1 || abs(q_y) > 1 ||
                abs(q_z) > 1 || abs(q_w) > 1) {
                std::cout << "odomtry pose: "
                          << "x: " << pose_x << " y: " << pose_y << " z: " << pose_z << " odomtry orientation: "
                          << " q_x: " << q_x << " q_y: " << q_y << " q_z: " << q_z << " q_w: " << q_w << std::endl;
            }

            m_odometry.pose.positionX = pose_x;
            m_odometry.pose.positionY = pose_y;
            m_odometry.pose.positionZ = pose_z;

            m_odometry.pose.orientationW = q_w;
            m_odometry.pose.orientationX = q_x;
            m_odometry.pose.orientationY = q_y;
            m_odometry.pose.orientationZ = q_z;

        } else if (message->topic == pointcloudTopic) {
            int32_t height = message->data()["height"]->as<int32_t>();
            int32_t width = message->data()["width"]->as<int32_t>();
            int32_t row_step = message->data()["row_step"]->as<int32_t>();
            int32_t point_step = message->data()["point_step"]->as<int32_t>();

            bool is_dense = message->data()["is_dense"]->as<bool>();

            const auto array = message->data()["data"];

            /*
                  std::cout << "point cloud height,width,point_step:" << height << ","
                            << width << ", " << point_step
                            << " row_step*height: " << row_step * height
                            << " array->size(): " << array->size() << std::endl;

                  std::cout << "is_dense: " << std::boolalpha << is_dense <<
               std::endl;
            */
            m_pointcloud.pointsMatrix.resize(3, height * width);

            int colIndex = 0;

            float x, y, z;
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    // Calculate the start index of the current point
                    size_t startIndex = i * row_step + j * point_step;

                    x = array[startIndex]->as<float>();
                    y = array[startIndex + 4]->as<float>();
                    z = array[startIndex + 8]->as<float>();
                    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                        // std::cout << "x,y,z: " << x << "," << y << "," << z << std::endl;

                        m_pointcloud.pointsMatrix(0, colIndex) = x;
                        m_pointcloud.pointsMatrix(1, colIndex) = y;
                        m_pointcloud.pointsMatrix(2, colIndex) = z;
                        colIndex++;
                    }
                }
            }

            // Eigen::SparseMatrix<float> sparseMat =
            //     m_pointcloud.pointsMatrix.sparseView();

            // vtkNew<vtkPoints> vtkpointcloudInWorld;
            // vtkNew<vtkPolyData> polyData;

            m_aligner_ptr->setPointcloud(m_pointcloud);
            m_aligner_ptr->setGimbalInBodyFrame(m_mount_control);
            m_aligner_ptr->setOdomInWorldFrame(m_odometry);
            m_aligner_ptr->setTofInGimbalFrame(TofInGimbalFrame);

            TimeStampedPoints pointcloudInWorldDownSampled = m_aligner_ptr->getPointCloudInWorldCoordinate();

            for (std::size_t i = 0; i < pointcloudInWorldDownSampled.pointsMatrix.cols(); i++) {
                vtkpointcloudInWorld->InsertNextPoint(
                    pointcloudInWorldDownSampled.pointsMatrix.col(i).x(),
                    pointcloudInWorldDownSampled.pointsMatrix.col(i).y(),
                    pointcloudInWorldDownSampled.pointsMatrix.col(i).z()
                );
            }
        }
    }
    polyData->SetPoints(vtkpointcloudInWorld);
    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    writer->SetFileName(std::string(std::to_string(m_aligner_ptr->getNumberOfPointTodecimate()) + "_map.ply").c_str());
    writer->SetInputData(polyData);
    writer->SetFileTypeToBinary();
    // writer->SetFileTypeToASCII();
    writer->Write();
    return;
}

TimeStampedPose ROSProcesser::getOdometry()
{
    return m_odometry;
}

TimeStampedPose ROSProcesser::getMountControlOrientation()
{
    return m_mount_control;
}

TimeStampedPoints ROSProcesser::getPointCloud()
{
    return m_pointcloud;
}
