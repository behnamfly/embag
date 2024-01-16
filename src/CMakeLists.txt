set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(eigen3 REQUIRED)

MESSAGE("EIGEN3_FOUND: " ${EIGEN3_FOUND})
MESSAGE("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
MESSAGE("EIGEN3_VERSION: " ${EIGEN3_VERSION})
MESSAGE("EIGEN3_VERSION_STRING: " ${EIGEN3_VERSION_STRING})

include_directories(${EIGEN3_INCLUDE_DIR})


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Qml Quick Widgets Gui QuickControls2 Xml Sql PrintSupport Svg  Network )
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Qml Quick Widgets Gui QuickControls2 Xml Sql PrintSupport Svg  Network )

message(STATUS "QT version: "    ${QT_VERSION_MAJOR}.${QT_VERSION_MINOR})
message(STATUS "QT dir" ${Qt${QT_VERSION_MAJOR}_DIR})


find_package(VTK REQUIRED)
message( "VTK_FOUND: "    ${VTK_FOUND})
message( "VTK_VERSION: "    ${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION} )

find_package(PCL  REQUIRED)
message( "PCL_FOUND: "    ${PCL_FOUND})
message( "PCL_VERSION: "    ${PCL_VERSION})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message( "PCL_LIBRARIES: "    ${PCL_LIBRARIES})
message( "PCL_INCLUDE_DIRS: "    ${PCL_INCLUDE_DIRS})
message( "PCL_LIBRARY_DIRS: "    ${PCL_LIBRARY_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

if (NOT  ${EXPLORE_PUBLIC_RELEASE})
    add_executable(align_pointcloud src/align_pointcloud.cpp src/pointcloudaligner.cpp src/pointcloudaligner.hpp)
    target_link_libraries(align_pointcloud PRIVATE  Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Qml Qt${QT_VERSION_MAJOR}::Quick Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::QuickControls2 Qt${QT_VERSION_MAJOR}::Xml Qt${QT_VERSION_MAJOR}::Svg Qt${QT_VERSION_MAJOR}::PrintSupport ${VTK_LIBRARIES} ${PCL_LIBRARIES})
    vtk_module_autoinit(TARGETS align_pointcloud MODULES ${VTK_LIBRARIES})
	


    add_executable(align_pointcloud_gimbal src/align_pointcloud_gimbal.cpp src/pointcloudaligner.cpp src/pointcloudaligner.hpp  src/data_types.hpp)
    target_link_libraries(align_pointcloud_gimbal PRIVATE  Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Qml Qt${QT_VERSION_MAJOR}::Quick Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::QuickControls2 Qt${QT_VERSION_MAJOR}::Xml Qt${QT_VERSION_MAJOR}::Svg Qt${QT_VERSION_MAJOR}::PrintSupport ${VTK_LIBRARIES} ${PCL_LIBRARIES} )
    vtk_module_autoinit(TARGETS align_pointcloud_gimbal MODULES ${VTK_LIBRARIES})


    add_executable(main src/main.cpp src/pointcloudaligner.cpp src/pointcloudaligner.hpp)
    target_link_libraries(main PRIVATE  Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Qml Qt${QT_VERSION_MAJOR}::Quick Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::QuickControls2 Qt${QT_VERSION_MAJOR}::Xml Qt${QT_VERSION_MAJOR}::Svg Qt${QT_VERSION_MAJOR}::PrintSupport ${VTK_LIBRARIES} ${PCL_LIBRARIES})
    vtk_module_autoinit(TARGETS main MODULES ${VTK_LIBRARIES})
  


    add_executable(test_pointcloudaligner src/test_pointcloudaligner.cpp src/pointcloudaligner.cpp src/pointcloudaligner.hpp  src/data_types.hpp)
    target_link_libraries(test_pointcloudaligner PRIVATE  Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Qml Qt${QT_VERSION_MAJOR}::Quick Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::QuickControls2 Qt${QT_VERSION_MAJOR}::Xml Qt${QT_VERSION_MAJOR}::Svg Qt${QT_VERSION_MAJOR}::PrintSupport ${VTK_LIBRARIES} ${PCL_LIBRARIES} )


endif()





