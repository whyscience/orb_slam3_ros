/**
 *
 * Common functions and variables across all modes (mono/stereo, with or w/o imu)
 *
 */

#include "common.h"

// Variables for ORB-SLAM3
ORB_SLAM3::System *pSLAM;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

// Variables for ROS 2
std::string world_frame_id, cam_frame_id, imu_frame_id;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_keypoints_pub;
image_transport::Publisher tracking_img_pub;

//////////////////////////////////////////////////
// Main functions
//////////////////////////////////////////////////

std::string getTimeStamp()
{
    using std::chrono::system_clock;
    system_clock::time_point tp = system_clock::now();
    time_t raw_time = system_clock::to_time_t(tp);

    // tm*使用完后不用delete，因为tm*是由localtime创建的，并且每个线程中会有一个
    struct tm *time_info = std::localtime(&raw_time);
    char buffer[80];

    // strftime(buffer, sizeof(buffer), "%H:%M:%S ", time_info);
    strftime(buffer, sizeof(buffer), "%T", time_info);
    return buffer;
}

bool save_map_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    auto time_str = getTimeStamp() + "_map";
    res->success = pSLAM->SaveMap(time_str /*req->name*/);

    if (res->success)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map was saved as %s.osa", time_str.c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Map could not be saved.");

    return res->success;
}

bool save_traj_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    auto time_str = getTimeStamp();
    const std::string cam_traj_file = /*req->name*/ time_str + "_cam_traj.txt";
    const std::string kf_traj_file = /*req->name*/ time_str + "_kf_traj.txt";

    try
    {
        pSLAM->SaveTrajectoryEuRoC(cam_traj_file);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        res->success = false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
        res->success = false;
    }

    if (!res->success)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Estimated trajectory could not be saved.");

    return res->success;
}

void setup_services(rclcpp::Node::SharedPtr node, std::string node_name)
{
    static auto save_map_service = node->create_service<std_srvs::srv::SetBool>(node_name + "/save_map", save_map_srv);
    static auto save_traj_service =
            node->create_service<std_srvs::srv::SetBool>(node_name + "/save_traj", save_traj_srv);
}

void setup_publishers(rclcpp::Node::SharedPtr node, image_transport::ImageTransport &image_transport,
                      std::string node_name)
{
    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", 1);

    tracked_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", 1);

    tracked_keypoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_key_points", 1);

    all_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", 1);

    tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 1);

    kf_markers_pub = node->create_publisher<visualization_msgs::msg::Marker>(node_name + "/kf_markers", 1000);

    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO ||
        sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", 1);
    }
}

void publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
        return;

    // Common topics
    publish_camera_pose(Twc, msg_time);
    publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

    publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);

    publish_keypoints(pSLAM->GetTrackedMapPoints(), pSLAM->GetTrackedKeyPoints(), msg_time);

    publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
    publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
    publish_kf_markers(pSLAM->GetAllKeyframePoses(), msg_time);

    // IMU-specific topics
    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO ||
        sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        // Body pose and translational velocity can be obtained from ORB-SLAM3
        Sophus::SE3f Twb = pSLAM->GetImuTwb();
        Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

        // IMU provides body angular velocity in body frame (Wbb) which is transformed to world frame (Wwb)
        Sophus::Matrix3f Rwb = Twb.rotationMatrix();
        Eigen::Vector3f Wwb = Rwb * Wbb;

        publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
        publish_body_odom(Twb, Vwb, Wwb, msg_time);
    }
}

void publish_body_odom(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body,
                       rclcpp::Time msg_time)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.child_frame_id = imu_frame_id;
    odom_msg.header.frame_id = world_frame_id;
    odom_msg.header.stamp = msg_time;

    odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

    odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
    odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
    odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
    odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

    odom_msg.twist.twist.linear.x = Vwb_E3f.x();
    odom_msg.twist.twist.linear.y = Vwb_E3f.y();
    odom_msg.twist.twist.linear.z = Vwb_E3f.z();

    odom_msg.twist.twist.angular.x = ang_vel_body.x();
    odom_msg.twist.twist.angular.y = ang_vel_body.y();
    odom_msg.twist.twist.angular.z = ang_vel_body.z();

    odom_pub->publish(odom_msg);
}

void publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id;
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Tcw_SE3f.translation().x();
    pose_msg.pose.position.y = Tcw_SE3f.translation().y();
    pose_msg.pose.position.z = Tcw_SE3f.translation().z();

    pose_msg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    pose_msg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    pose_msg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    pose_msg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pose_pub->publish(pose_msg);
}

void publish_tf_transform(Sophus::SE3f T_SE3f, std::string frame_id, std::string child_frame_id, rclcpp::Time msg_time)
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = msg_time;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;

    Eigen::Quaternionf q = T_SE3f.unit_quaternion();
    transform_stamped.transform.rotation.w = q.w();
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();

    transform_stamped.transform.translation.x = T_SE3f.translation().x();
    transform_stamped.transform.translation.y = T_SE3f.translation().y();
    transform_stamped.transform.translation.z = T_SE3f.translation().z();

    tf_broadcaster->sendTransform(transform_stamped);
}

void publish_tracking_img(cv::Mat image, rclcpp::Time msg_time)
{
    std_msgs::msg::Header header;

    header.stamp = msg_time;

    header.frame_id = world_frame_id;

    const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
            cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    tracking_img_pub.publish(rendered_image_msg);
}

void publish_keypoints(std::vector<ORB_SLAM3::MapPoint *> tracked_map_points,
                       std::vector<cv::KeyPoint> tracked_keypoints, rclcpp::Time msg_time)
{
    std::vector<cv::KeyPoint> finalKeypoints;

    int numKFs = tracked_keypoints.size();

    if (tracked_keypoints.empty())
        return;

    for (size_t i = 0; i < tracked_map_points.size(); i++)
    {
        if (tracked_map_points[i])
        { // if the MapPoint pointer is not nullptr
            finalKeypoints.push_back(tracked_keypoints[i]);
        }
    }

    sensor_msgs::msg::PointCloud2 cloud = keypoints_to_pointcloud(finalKeypoints, msg_time);

    tracked_keypoints_pub->publish(cloud);
}


void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint *> tracked_points, rclcpp::Time msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);

    tracked_mappoints_pub->publish(cloud);
}

void publish_all_points(std::vector<ORB_SLAM3::MapPoint *> map_points, rclcpp::Time msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);

    all_mappoints_pub->publish(cloud);
}

// More details: http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html
void publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, rclcpp::Time msg_time)
{
    int numKFs = vKFposes.size();
    if (numKFs == 0)
        return;

    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = world_frame_id;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration(0, 0); // 5 seconds and 0 nanoseconds

    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    for (int i = 0; i <= numKFs; i++)
    {
        geometry_msgs::msg::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }

    kf_markers_pub->publish(kf_markers);
}

//////////////////////////////////////////////////
// Miscellaneous functions
//////////////////////////////////////////////////

sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(std::vector<cv::KeyPoint> &keypoints, rclcpp::Time msg_time)
{
    const int num_channels = 3; // x y z

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = keypoints.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        float data_array[num_channels] = {
                keypoints[i].pt.x, keypoints[i].pt.y,
                0.0f // Z value is 0 for 2D keypoints
        };

        memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
    return cloud;
}

sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                                                     rclcpp::Time msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            tf2::Vector3 point_translation(P3Dw.x(), P3Dw.y(), P3Dw.z());

            float data_array[num_channels] = {(float) point_translation.x(), (float) point_translation.y(),
                                              (float) point_translation.z()};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}

cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f)
{
    cv::Mat T_cvmat;

    Eigen::Matrix4f T_Eig3f = T_SE3f.matrix();
    cv::eigen2cv(T_Eig3f, T_cvmat);

    return T_cvmat;
}

geometry_msgs::msg::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    geometry_msgs::msg::Transform tf_transform;

    tf_transform.rotation.w = T_SE3f.unit_quaternion().w();
    tf_transform.rotation.x = T_SE3f.unit_quaternion().x();
    tf_transform.rotation.y = T_SE3f.unit_quaternion().y();
    tf_transform.rotation.z = T_SE3f.unit_quaternion().z();

    tf_transform.translation.x = t_vec(0);
    tf_transform.translation.y = t_vec(1);
    tf_transform.translation.z = t_vec(2);

    return tf_transform;
}
