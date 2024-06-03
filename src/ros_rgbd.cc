/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
 *
 */

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr &msgRGB, const sensor_msgs::msg::Image::SharedPtr &msgD);
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("orb_slam3");

    if (argc > 1)
    {
        RCLCPP_WARN(node->get_logger(), "Arguments supplied via command line are ignored.");
    }

    auto node_name = node->get_name();
    image_transport::ImageTransport image_transport(node);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    std::string voc_file, settings_file;
    node->declare_parameter<std::string>("voc_file", default_voc_file);
    node->declare_parameter<std::string>("settings_file", std::string(PROJECT_SOURCE_DIR) + "/config/RGB-D/RealSense_D435i.yaml");
    node->get_parameter("voc_file", voc_file);
    node->get_parameter("settings_file", settings_file);

    std::string world_frame_id, cam_frame_id;
    node->declare_parameter<std::string>("world_frame_id", "map");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->get_parameter("world_frame_id", world_frame_id);
    node->get_parameter("cam_frame_id", cam_frame_id);

    bool enable_pangolin{};
    node->declare_parameter<bool>("enable_pangolin", true);
    node->get_parameter("enable_pangolin", enable_pangolin);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb;

    std::string rgb_img_topic = "/camera/rgb/image_raw";
    std::string depth_img_topic = "/camera/depth_registered/image_raw";
    std::string imu_topic = "/imu/data";

    //debug code
    // rgb_img_topic = "/cam0/image_raw";
    // depth_img_topic = "/cam1/image_raw";
    // imu_topic = "/imu0";

    auto sub_rgb_img =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, rgb_img_topic);
    auto sub_depth_img = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            node, depth_img_topic);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
            approximate_sync_policy;
    auto syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
            approximate_sync_policy(10), *sub_rgb_img, *sub_depth_img);
    syncApproximate->registerCallback(&ImageGrabber::GrabRGBD, &igb);

    setup_publishers(node, image_transport, node_name);
    setup_services(node, node_name);

    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr &msgRGB,
                            const sensor_msgs::msg::Image::SharedPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackRGBD()
    /*Sophus::SE3f Tcw =*/
    pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, rclcpp::Time(cv_ptrRGB->header.stamp).seconds());

    rclcpp::Time msg_time = cv_ptrRGB->header.stamp;

    publish_topics(msg_time);
}
