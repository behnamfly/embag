cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0079 NEW)

project(embag)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Boost 1.74 COMPONENTS date_time filesystem system iostreams REQUIRED)

find_package(VTK REQUIRED)

find_package(BZip2 REQUIRED)

if(BZIP2_FOUND)
    message(${BZIP2_LIBRARIES} )
endif()


#find_package(LZ4 REQUIRED)
find_package(lz4 CONFIG REQUIRED)



# if(EXISTS "$ENV{HOME}/usr/lib/cmake/opencv4")
# 	set(OpenCV_DIR "$ENV{HOME}/usr/lib/cmake/opencv4")
# endif()

find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
	message("OpenCV_DIR: " ${OpenCV_DIR})
	message("OpenCV_VERSION: " ${OpenCV_VERSION})
	message("OpenCV_Include_DIRS: " ${OpenCV_Include_DIRS})
	message("OpenCV_LIBS: " ${OpenCV_LIBS})
else()
	message("OpenCV not found")
endif()



find_package(Eigen3)
IF(${EIGEN3_FOUND})
	MESSAGE("EIGEN3_FOUND: " ${EIGEN3_FOUND})
	MESSAGE("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
	MESSAGE("EIGEN3_VERSION: " ${EIGEN3_VERSION})
	MESSAGE("EIGEN3_VERSION_STRING: " ${EIGEN3_VERSION_STRING})
	include_directories(${EIGEN3_INCLUDE_DIR})

	
ENDIF()


set(SRC_FILES
src/wrapper.cpp 
src/sensor_data_interface.cpp 
src/pointcloudaligner.cpp 
src/sensor.hpp
src/ros_processer.cpp
src/mavlink_data.hpp)


set(HEADER_FILES
src/ArgumentsParser.hpp 
src/data_types.hpp 
src/wrapper.hpp 
src/sensor_data_interface.hpp 
src/pointcloudaligner.hpp 
src/sensor.hpp 
src/ros_processer.hpp
src/mavlink_data.hpp
src/circular_buffer.hpp
)



include_directories(${Boost_INCLUDE_DIRS})
message(${Boost_LIBRARIES} )
include_directories(lib)
file(GLOB LIB "lib/*.cc")
add_executable(poitncloud_aligner src/main.cpp ${LIB} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(poitncloud_aligner PRIVATE  ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${OpenCV_LIBS} ${VTK_LIBRARIES}  lz4::lz4)
vtk_module_autoinit(TARGETS poitncloud_aligner MODULES ${VTK_LIBRARIES})







# src/align_pointcloud.cpp
# src/align_pointcloud_gimbal.cpp
# src/test_pointcloudaligner.cpp
