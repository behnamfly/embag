#include <filesystem>
#include <string>
#include <vector>

#include "../../tools/src/ArgumentsParser.hpp"
#include "../lib/embag.h"
#include "../lib/view.h"

#include "pointcloudaligner.hpp"
#include "ros_processer.hpp"

int readingBag(const std::string& bagFileName, int decimation = 20)
{
    std::filesystem::path full_path = std::filesystem::absolute(bagFileName);

    // Check if the file exists
    if (std::filesystem::exists(full_path)) {
        std::cout << "The input bag file is: " << full_path << std::endl;
    } else {
        std::cout << "Error, The input bag does not exist: " << full_path << std::endl;
        return 1;
    }

    Embag::View view{};
    view.addBag(bagFileName);
    ROSProcesser* ros_processer = new ROSProcesser;
    std::unique_ptr<PointCloudAligner> aligner_ptr = std::make_unique<PointCloudAligner>();
    aligner_ptr->setNumberOfPointTodecimate(decimation);
    aligner_ptr->setMaxDistanceClamping(4);
    aligner_ptr->setMinDistanceClamping(0.4);
    ros_processer->setPointCloudAligner(std::move(aligner_ptr));
    ros_processer->processBagData(view);
}

int main(int argc, char** argv)
{

    ArgumentsParser input(argc, argv);
    if (input.argExists("-h") || input.argExists("--help") || argc == 1) {
        std::cerr << "usage: pointcloud_aligner.exe "
                     " -i <input-bag-file> "
                     " -d <decimation> "
                     "\n Examples of run:"
                     "\n pointcloud_aligner.exe -i file.bag  -d 20"
                  << std::endl;
        return 1;
    }

    std::string inputBagFile = input.getArg("-i");

    if (inputBagFile.empty()) {
        std::cerr << "no intput bag file is given, please specify the input file, -i input-bag-file> " << std::endl;
        return 1;
    }

    std::string strDeafultDecimation = "20";
    std::string strDecimation = input.getArg("-d");
    if (strDecimation.empty()) {
        std::cout << "decimation is not given, default value " << strDeafultDecimation << " will be used" << std::endl;
        strDecimation = strDeafultDecimation;
    }

    // std::vector<std::string> bags = {
    //     "bags/session_2023-12-13-13-35-23_0.bag", "bags/session_2023-12-13-13-35-23_0_sliced.bag",
    //     "bags/session_2023-12-13-13-37-40_1.bag", "bags/session_2023-12-13-13-39-50_2.bag"
    // };

    // std::string bag_file = "../../ASIO-Explore/embag/" + bags[2];
    // std::cout << "bag_file:" << bag_file << std::endl;

    int decimation = std::stoi(strDecimation);
    readingBag(inputBagFile, decimation);
    return 0;
}
