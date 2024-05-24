/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
 *
 */

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft,
                    const sensor_msgs::msg::Image::ConstSharedPtr msgRight);
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

    std::string voc_file, settings_file;
    node->declare_parameter<std::string>("voc_file", "file_not_set");
    node->declare_parameter<std::string>("settings_file", "file_not_set");
    node->get_parameter("voc_file", voc_file);
    node->get_parameter("settings_file", settings_file);

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        RCLCPP_ERROR(node->get_logger(), "Please provide voc_file and settings_file in the launch file");
        rclcpp::shutdown();
        return 1;
    }

    std::string world_frame_id, cam_frame_id;
    node->declare_parameter<std::string>("world_frame_id", "map");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->get_parameter("world_frame_id", world_frame_id);
    node->get_parameter("cam_frame_id", cam_frame_id);

    bool enable_pangolin{};
    node->declare_parameter<bool>("enable_pangolin", true);
    node->get_parameter("enable_pangolin", enable_pangolin);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb;

    auto sub_img_left =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, "/camera/left/image_raw");
    auto sub_img_right =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, "/camera/right/image_raw");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), *sub_img_left, *sub_img_right);
    sync.registerCallback(std::bind(&ImageGrabber::GrabStereo, &igb, std::placeholders::_1, std::placeholders::_2));

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

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft,
                              const sensor_msgs::msg::Image::ConstSharedPtr msgRight)
{
    auto msg_time = msgLeft->header.stamp;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackStereo()
    Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, rclcpp::Time(msg_time).seconds());

    publish_topics(msg_time);
}
