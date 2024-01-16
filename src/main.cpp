// #include "../lib/span.hpp"
// #include <array>
// #include <iostream>
// #include <vector>

// std::ptrdiff_t size(nonstd::span<const int> spn) { return spn.size(); }

// int main() {

//   int arr[] = {
//       1,
//   };

//   std::cout << "C-array:" << size(arr) << " array:"
//             << size(std::array<int, 2>{
//                    1,
//                    2,
//                })
//             << " vector:"
//             << size(std::vector<int>{
//                    1,
//                    2,
//                    3,
//                });
// }

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/opencv.hpp>

#include <vtkPLYWriter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include "../lib/embag.h"
#include "../lib/view.h"
#include "circular_buffer.hpp"
#include "pointcloudaligner.hpp"
#include "ros_processer.hpp"
#include "sensor.hpp"
#include "sensor_data_interface.hpp"

int readingBagDirect()
{
    //   Embag::Bag bag_{"../../bags/test.bag"};

    //   Embag::Bag bag_{
    //       "/media/behnam/FLYBOTIX1/bags/session_2023-12-13-13-37-40_1.bag"};

    //   std::set<std::string> topic_set;

    //   for (const auto &topic : bag_.topics()) {
    //     topic_set.emplace(topic);
    //     std::cout << topic << std::endl;
    //   }

    std::vector<std::string> bags = {
        "bags/session_2023-12-13-13-35-23_0.bag", "bags/session_2023-12-13-13-35-23_0_sliced.bag",
        "bags/session_2023-12-13-13-37-40_1.bag", "bags/session_2023-12-13-13-39-50_2.bag"
    };

    std::string bag_file = "../../" + bags[1];

    Embag::View view_{bag_file};
    Embag::Bag bag_{bag_file};

    auto def = bag_.msgDefForTopic("/camera_bridge/cam_down/image_mono");

    Embag::View view{};

    view.addBag(bag_file);

    vtkNew<vtkPoints> vtkpointcloudInWorld;
    vtkNew<vtkPolyData> polyData;
    int cloud_id = 0;

    for (const auto &message : view.getMessages(
             {"/royale_camera_driver/point_cloud", "/mavros/odometry/in", "/mavros/mount_control/orientation"}
         )) {

        std::cout << std::fixed << message->timestamp.to_sec() << " : " << message->topic << std::endl;

        std::cout << "time is " << message->timestamp.secs << "." << message->timestamp.nsecs << std::endl;

        if (message->topic == "/mavros/mount_control/orientation") {

            double x, y, z, w;

            x = message->data()["x"]->as<double>();
            y = message->data()["y"]->as<double>();
            z = message->data()["z"]->as<double>();
            w = message->data()["w"]->as<double>();

            /*
                  std::cout << "x: " << x << " y: " << y << " z: " << z << " w: " <<
               w<< std::endl;
            */
        } else if (message->topic == "/mavros/odometry/in") {

            std::cout << message->data()["child_frame_id"]->as<std::string>() << std::endl;

            double x, y, z;

            x = message->data()["pose"]->get("pose")->get("position")->get("x")->as<double>();
            y = message->data()["pose"]->get("pose")->get("position")->get("y")->as<double>();
            z = message->data()["pose"]->get("pose")->get("position")->get("z")->as<double>();

            std::cout << "pose: "
                      << "x: " << x << " y: " << y << " z: " << z << std::endl;

            // assert(pose[0]->getType() == Embag::RosValue::Type::float64);

            // for (auto item = pose->beginValues<Embag::RosValue::Pointer>();
            //      item != pose->endValues<Embag::RosValue::Pointer>(); item++) {

            //   std::cout << (*item)->as<double>() << std::endl;
            // }
        } else if (message->topic == "/royale_camera_driver/point_cloud") {
            int32_t height = message->data()["height"]->as<int32_t>();
            int32_t width = message->data()["width"]->as<int32_t>();
            int32_t row_step = message->data()["row_step"]->as<int32_t>();
            int32_t point_step = message->data()["point_step"]->as<int32_t>();

            bool is_dense = message->data()["is_dense"]->as<bool>();

            const auto array = message->data()["data"];

            // auto fields_name =
            //     message->data()["fields"]->get("name")->as<std::string>();

            // std::cout << "fields_name: " << fields_name << std::endl;

            std::cout << "point cloud height,width,point_step:" << height << ", " << width << ", " << point_step
                      << " row_step*height: " << row_step * height << " array->size(): " << array->size() << std::endl;

            std::cout << "is_dense" << is_dense << std::endl;

            for (auto item = array->beginValues<Embag::RosValue::Pointer>();
                 item != array->endValues<Embag::RosValue::Pointer>(); item++) {

                // std::cout << (*item)->as<uint8_t>() << std::endl;
            }

            std::cout << "sizeof(float): " << sizeof(float) << std::endl;
            float x, y, z;
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    // Calculate the start index of the current point
                    size_t startIndex = i * row_step + j * point_step;

                    x = array[startIndex]->as<float>();
                    y = array[startIndex + 4]->as<float>();
                    z = array[startIndex + 8]->as<float>();

                    // std::cout << "x,y,z: " << x << "," << y << "," << z << std::endl;

                    vtkpointcloudInWorld->InsertNextPoint(x, y, z);
                }
            }

            cloud_id++;
        }
        polyData->SetPoints(vtkpointcloudInWorld);
        vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
        writer->SetFileName(std::string(std::to_string(cloud_id) + "_map.ply").c_str());
        writer->SetInputData(polyData);  // Assuming 'polyData' is your vtkPolyData object
        writer->SetFileTypeToBinary();
        writer->Write();

        // std::cout <<
        // message->data()["fun_array"][0]["fun_field"]->as<std::string>()
        //           << std::endl;

        // std::cout <<
        // message->data()["fun_array"][0]["fun_field"]->as<std::string>()
        //           << std::endl;
    }
    // for (const auto &filename : vm["bag"].as<std::vector<std::string>>()) {
    //   std::cout << "Opening " << filename << std::endl;
    //   view.addBag(filename);
    // }

    const auto start_time = view.getStartTime();
    const auto end_time = view.getEndTime();

    std::cout << "Start time is " << start_time.secs << "." << start_time.nsecs << std::endl;
    std::cout << "End time is " << end_time.secs << "." << end_time.nsecs << std::endl;

    int img_id = 0;

    for (const auto &message : view.getMessages()) {
        if (message->topic == "/camera_bridge/cam_down/image_mono") {
            std::cout << message->data()["header"]["seq"]->as<uint32_t>() << std::endl;

            std::cout << "stamp:" << message->timestamp.secs << "," << message->timestamp.nsecs << std::endl;

            // std::cout << message->data()["encoding"]->as<std::string>() <<
            // std::endl;

            auto step = message->data()["step"]->as<uint32_t>();
            // std::cout << step << std::endl;

            auto height = message->data()["height"]->as<uint32_t>();
            // std::cout << height << std::endl;

            int rows = height;  // Number of rows
            int cols = step;    // Number of columns

            cv::Mat mat(rows, cols, CV_8UC1);

            auto matrixSize = step * height;

            // auto data = message->data()["data"]->as<uint8_t *>();

            const auto array = message->data()["data"];
            // std::cout << array->size() << std::endl;

            // for (std::size_t i = 0; i < matrixSize; i++) {

            // }

            assert(array[0]->getType() == Embag::RosValue::Type::uint8);

            for (auto item = array->beginValues<Embag::RosValue::Pointer>();
                 item != array->endValues<Embag::RosValue::Pointer>(); item++) {

                // std::cout << (*item)->as<uint8_t>() << std::endl;
            }

            auto begin = array->beginValues<Embag::RosValue::Pointer>();
            auto it = array->beginValues<Embag::RosValue::Pointer>();  // Iterator to the beginning of your data

            auto end = array->endValues<Embag::RosValue::Pointer>();

            // for (int y = 0; y < mat.rows; ++y) {
            //   for (int x = 0; x < mat.cols; ++x) {
            //     // Ensure that we don't go past the end of the data
            //     if (it == end) {
            //       break;
            //     }
            //     mat.at<uint8_t>(y, x) = int(*it.get());
            //     ++it;
            //   }
            // }

            // std::copy(begin, end, mat.data);

            // for (auto item =
            // dynamic_uint64_array->beginValues<Embag::RosValue::Pointer>(); item !=
            // dynamic_uint64_array->endValues<Embag::RosValue::Pointer>(); item++) {
            //   ASSERT_EQ((*item)->as<uint64_t>(), (uint64_t) index * inner_index++);
            // }

            for (size_t i = 0; i < array->size(); ++i) {

                // std::cout << static_cast<int>(array[i]->as<uint8_t>()) << std::endl;
            }

            for (int y = 0; y < mat.rows; ++y) {
                for (int x = 0; x < mat.cols; ++x) {

                    mat.at<uint8_t>(y, x) = array[y * mat.cols + x]->as<uint8_t>();
                }
            }
            cv::imwrite(std::to_string(img_id) + ".jpg", mat);
            img_id++;
        }
    }

    return 0;
}

class ROSOdometryType
{
};
class ROSOrientationType
{
};
class ROSPointCloudType
{
};

class MavlinkOdometryType
{
};
class MavlinkOrientationType
{
};
class MavlinkPointCloudType
{
};

void preProcessingData()
{  // ros data
    ROSPointCloudType rosPointCloud;
    auto rosPointcloudData = std::make_shared<PointCloudWrapperImpl<ROSPointCloudType>>(rosPointCloud);
    SensorDataInterface *sensorInterfaceROS = new ROSDataInterface();
    sensorInterfaceROS->addPointCloudData(rosPointcloudData);
    sensorInterfaceROS->processPointCloud();

    // mavlink data

    MavlinkPointCloudType mavlinkPointCloud;
    auto mavLinkPointcloudData = std::make_shared<PointCloudWrapperImpl<MavlinkPointCloudType>>(mavlinkPointCloud);

    SensorDataInterface *sensorInterfaceMavlink = new MavlinkDataInterface();
    sensorInterfaceMavlink->addPointCloudData(mavLinkPointcloudData);
    sensorInterfaceMavlink->processPointCloud();
}

void readingBag()
{
    std::vector<std::string> bags = {
        "bags/session_2023-12-13-13-35-23_0.bag", "bags/session_2023-12-13-13-35-23_0_sliced.bag",
        "bags/session_2023-12-13-13-37-40_1.bag", "bags/session_2023-12-13-13-39-50_2.bag"
    };

    std::string bag_file = "../../ASIO-Explore/embag/" + bags[2];
    std::cout << "bag_file:" << bag_file << std::endl;

    Embag::View view{};

    view.addBag(bag_file);

    ROSProcesser *ros_processer = new ROSProcesser;

    int decimation = 20;

    std::unique_ptr<PointCloudAligner> aligner_ptr = std::make_unique<PointCloudAligner>();

    aligner_ptr->setNumberOfPointTodecimate(decimation);
    aligner_ptr->setMaxDistanceClamping(4);
    aligner_ptr->setMinDistanceClamping(0.4);
    ros_processer->setPointCloudAligner(std::move(aligner_ptr));
    ros_processer->processBagData(view);
}
void readMP4Mavlink()
{

    int n = 1;
    std::vector<MavlinkOdometryType> odoms(4006);
    std::vector<MavlinkOrientationType> orientations(2679);
    std::vector<MavlinkPointCloudType> clouds(629);

    CircularBuffer<MavlinkPointCloudType> buffClouds(10);
    CircularBuffer<MavlinkOdometryType> buffOdoms(10 * 6);
    CircularBuffer<MavlinkOrientationType> buffOrientations(10 * 2);

    for (std::size_t i = 0; i < 100; i++) {
        buffOrientations.push(orientations[i]);
        buffOdoms.push(odoms[i]);
        buffClouds.push(clouds[i]);
    }
}
int main()
{
    readingBag();
    return 0;
}
