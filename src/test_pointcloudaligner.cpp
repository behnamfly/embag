#include "fstream"
#include "pointcloudaligner.hpp"

Eigen::Matrix<float, 3, Eigen::Dynamic> CSVToEigenMatrix(const std::string& csvFilePath)
{
    std::string line;
    std::ifstream file(csvFilePath.c_str());  // Replace with your CSV file path

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
        // return 1;
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
    return points;
}

void writeEigenMatrixToCSV(const std::string& csvFilePath, const Eigen::Matrix<float, 3, Eigen::Dynamic>& points)
{  // Open a file in write mode.
    std::ofstream outFile(csvFilePath.c_str());
    std::cout << "writing " << points.cols() << " into the file" << std::endl;

    if (outFile.is_open()) {
        // Write each point to the file
        for (int i = 0; i < points.cols(); ++i) {
            // std::cout << points(0, i) << "," << points(1, i) << "," << points(2, i) << std::endl;

            outFile << points(0, i) << "," << points(1, i) << "," << points(2, i) << "\n";
        }

        outFile.close();  // Close the file after writing is done
        std::cout << "bar" << std::endl;

        std::cout << "point cloud written to " << csvFilePath << std::endl;
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

int main()
{

    PointCloudAligner aligner;
    aligner.setNumberOfPointTodecimate(5);

    TimeStampedPoints newPointcloud;
    newPointcloud.pointsMatrix = CSVToEigenMatrix("bunny.csv");
    aligner.setPointcloud(newPointcloud);
    aligner.decimation();

    writeEigenMatrixToCSV("decimated_points.csv", aligner.getDecimatedPoints().pointsMatrix);

    std::cout << "clamping" << std::endl;

    aligner.setMaxDistanceClamping(2);
    aligner.setMinDistanceClamping(0.4);

    aligner.clamping();
    writeEigenMatrixToCSV("decimated_clamped_points.csv", aligner.getDecimatedPoints().pointsMatrix);
}
