#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>


// ORB-SLAM3-specific libraries
#include "ImuTypes.h"
#include "System.h"

extern ORB_SLAM3::System *pSLAM;
extern std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
extern ORB_SLAM3::System::eSensor sensor_type;

extern std::string world_frame_id, cam_frame_id, imu_frame_id;
inline std::string default_voc_file = std::string(PROJECT_SOURCE_DIR) + "/orb_slam3/Vocabulary/ORBvoc.txt";//.bin

extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
extern image_transport::Publisher tracking_img_pub;

void setup_services(const rclcpp::Node::SharedPtr &, const std::string &);
void setup_publishers(const rclcpp::Node::SharedPtr &, image_transport::ImageTransport &, const std::string &);
void publish_topics(const rclcpp::Time &, const Eigen::Vector3f & = Eigen::Vector3f::Zero());

void publish_camera_pose(Sophus::SE3f, const rclcpp::Time &);
void publish_tracking_img(const cv::Mat &, const rclcpp::Time &);
void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint *>, const rclcpp::Time &);
void publish_keypoints(std::vector<ORB_SLAM3::MapPoint *>, std::vector<cv::KeyPoint>, const rclcpp::Time &);
sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(std::vector<cv::KeyPoint> &, const rclcpp::Time &);

void publish_all_points(std::vector<ORB_SLAM3::MapPoint *>, const rclcpp::Time &);
void publish_tf_transform(Sophus::SE3f, std::string, std::string, const rclcpp::Time &);
void publish_body_odom(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, const rclcpp::Time &);
void publish_kf_markers(std::vector<Sophus::SE3f>, const rclcpp::Time &);

bool save_map_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
                  std::shared_ptr<std_srvs::srv::SetBool::Response>);
bool save_traj_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
                   std::shared_ptr<std_srvs::srv::SetBool::Response>);

cv::Mat SE3f_to_cvMat(const Sophus::SE3f &);
geometry_msgs::msg::Transform SE3f_to_tfTransform(Sophus::SE3f);
sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *>, const rclcpp::Time &);

inline double
toSec(const builtin_interfaces::msg::Time &time) // do not convert to const rclcpp::Time &time, it is wrong!
{
    return time.sec + time.nanosec * 1e-9;
}
