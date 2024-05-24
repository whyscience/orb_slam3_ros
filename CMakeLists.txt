cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros)

# Make sure to set this path before building the
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation

# Must use C++17 to make it compatible with rclcpp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -Wno-deprecated-declarations -Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare -Wno-unused-but-set-variable -Wno-vla -Wno-reorder -Wno-type-limits -Wno-aggressive-loop-optimizations")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++17.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif ()
#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)

# Header file locations [C++ node]
include_directories(include) # Add .hpp, .h files from include/ros2_orb_slam3
include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${Pangolin_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin


# ORB-SLAM3 includes [VSLAM library]
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/orb_slam3
        ${PROJECT_SOURCE_DIR}/orb_slam3/include
        ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
        ${ament_INCLUDE_DIRS}
)


set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclpy
        std_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        nav_msgs
        visualization_msgs
        tf2_ros
        message_filters
        cv_bridge
        image_transport
        OpenCV
        Eigen3
        Pangolin
)

add_subdirectory(orb_slam3/Thirdparty/DBoW2)
add_subdirectory(orb_slam3/Thirdparty/g2o)
add_subdirectory(orb_slam3/Thirdparty/Sophus)

#* ORB SLAM3 as a shared library
# Whenever you add a new .h, .hpp or .cc, .cpp file in the ros2_orb_slam3/orb_slam3/include and os2_orb_slam3/orb_slam3/src directories, make sure to add them here as shown below
add_library(orb_slam3_lib SHARED
        orb_slam3/src/System.cc
        orb_slam3/src/Tracking.cc
        orb_slam3/src/LocalMapping.cc
        orb_slam3/src/LoopClosing.cc
        orb_slam3/src/ORBextractor.cc
        orb_slam3/src/ORBmatcher.cc
        orb_slam3/src/FrameDrawer.cc
        orb_slam3/src/Converter.cc
        orb_slam3/src/MapPoint.cc
        orb_slam3/src/KeyFrame.cc
        orb_slam3/src/Atlas.cc
        orb_slam3/src/Map.cc
        orb_slam3/src/MapDrawer.cc
        orb_slam3/src/Optimizer.cc
        orb_slam3/src/Frame.cc
        orb_slam3/src/KeyFrameDatabase.cc
        orb_slam3/src/Sim3Solver.cc
        orb_slam3/src/Viewer.cc
        orb_slam3/src/ImuTypes.cc
        orb_slam3/src/G2oTypes.cc
        orb_slam3/src/CameraModels/Pinhole.cpp
        orb_slam3/src/CameraModels/KannalaBrandt8.cpp
        orb_slam3/src/OptimizableTypes.cpp
        orb_slam3/src/MLPnPsolver.cpp
        orb_slam3/src/GeometricTools.cc
        orb_slam3/src/TwoViewReconstruction.cc
        orb_slam3/src/Config.cc
        orb_slam3/src/Settings.cc
        orb_slam3/include/System.h
        orb_slam3/include/Tracking.h
        orb_slam3/include/LocalMapping.h
        orb_slam3/include/LoopClosing.h
        orb_slam3/include/ORBextractor.h
        orb_slam3/include/ORBmatcher.h
        orb_slam3/include/FrameDrawer.h
        orb_slam3/include/Converter.h
        orb_slam3/include/MapPoint.h
        orb_slam3/include/KeyFrame.h
        orb_slam3/include/Atlas.h
        orb_slam3/include/Map.h
        orb_slam3/include/MapDrawer.h
        orb_slam3/include/Optimizer.h
        orb_slam3/include/Frame.h
        orb_slam3/include/KeyFrameDatabase.h
        orb_slam3/include/Sim3Solver.h
        orb_slam3/include/Viewer.h
        orb_slam3/include/ImuTypes.h
        orb_slam3/include/G2oTypes.h
        orb_slam3/include/CameraModels/GeometricCamera.h
        orb_slam3/include/CameraModels/Pinhole.h
        orb_slam3/include/CameraModels/KannalaBrandt8.h
        orb_slam3/include/OptimizableTypes.h
        orb_slam3/include/MLPnPsolver.h
        orb_slam3/include/GeometricTools.h
        orb_slam3/include/TwoViewReconstruction.h
        orb_slam3/include/SerializationUtils.h
        orb_slam3/include/Config.h
        orb_slam3/include/Settings.h
)

#set_target_properties(orb_slam3_lib PROPERTIES VERSION "${orb_slam3_lib_VERSION}")

ament_target_dependencies(orb_slam3_lib
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link libraries
target_link_libraries(orb_slam3_lib
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
        -lboost_system
        -lboost_serialization
        -lcrypto
)


# Find the .so files provided in ros2_orb_slam3/orb_slam3/Thirdparty projects
set(DBoW2_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so")
set(g2o_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so")

# Install .so files to the lib directory in the install space
install(FILES ${DBoW2_PATH} DESTINATION lib)
install(FILES ${g2o_PATH} DESTINATION lib)

## ROS node
add_executable(ros_mono src/ros_mono.cc src/common.cc)
ament_target_dependencies(ros_mono PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_mono PUBLIC orb_slam3_lib) # Link a node with the internal shared library

add_executable(ros_mono_inertial src/ros_mono_inertial.cc src/common.cc)
ament_target_dependencies(ros_mono_inertial PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_mono_inertial PUBLIC orb_slam3_lib)

add_executable(ros_stereo src/ros_stereo.cc src/common.cc)
ament_target_dependencies(ros_stereo PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_stereo PUBLIC orb_slam3_lib)

add_executable(ros_stereo_inertial src/ros_stereo_inertial.cc src/common.cc)
ament_target_dependencies(ros_stereo_inertial PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_stereo_inertial PUBLIC orb_slam3_lib)

add_executable(ros_rgbd src/ros_rgbd.cc src/common.cc)
ament_target_dependencies(ros_rgbd PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_rgbd PUBLIC orb_slam3_lib)

add_executable(ros_rgbd_inertial src/ros_rgbd_inertial.cc src/common.cc)
ament_target_dependencies(ros_rgbd_inertial PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ros_rgbd_inertial PUBLIC orb_slam3_lib)

# Install all the header files in package/package/include
install(DIRECTORY include/
        DESTINATION include
)

# Install our node and library
install(TARGETS orb_slam3_lib
        ros_mono
        ros_mono_inertial
        ros_stereo
        ros_stereo_inertial
        ros_rgbd
        ros_rgbd_inertial
        DESTINATION lib/${PROJECT_NAME}
)

# Tell downstream packages where to find the headers
ament_export_include_directories(include)

# Help downstream packages to find transitive dependencies
ament_export_dependencies(
        orb_slam3_lib
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
        orb_slam3/Vocabulary/
        DESTINATION share/${PROJECT_NAME}/orb_slam3/Vocabulary/
)
ament_package()