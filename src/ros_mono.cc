/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
 *
 */

#include "common.h"

using namespace std;

class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber() : Node("Mono")
    {
        this->declare_parameter<std::string>("voc_file", "file_not_set");
        this->declare_parameter<std::string>("settings_file", "file_not_set");
        this->declare_parameter<std::string>("world_frame_id", "map");
        this->declare_parameter<std::string>("cam_frame_id", "camera");
        this->declare_parameter<bool>("enable_pangolin", true);

        this->get_parameter("voc_file", voc_file_);
        this->get_parameter("settings_file", settings_file_);

        if (voc_file_ == "file_not_set" || settings_file_ == "file_not_set")
        {
            RCLCPP_ERROR(this->get_logger(), "Please provide voc_file and settings_file in the launch file");
            rclcpp::shutdown();
        }

        this->get_parameter("world_frame_id", world_frame_id);
        this->get_parameter("cam_frame_id", cam_frame_id);
        this->get_parameter("enable_pangolin", enable_pangolin_);

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        sensor_type = ORB_SLAM3::System::MONOCULAR;
        pSLAM = new ORB_SLAM3::System(voc_file_, settings_file_, sensor_type, enable_pangolin_);

        image_transport::ImageTransport it(this);
        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 1, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));

        setup_publishers(shared_from_this(), it, this->get_name());
        setup_services(shared_from_this(), this->get_name());
    }

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    std::string voc_file_, settings_file_;
    bool enable_pangolin_;
};

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, rclcpp::Time(msg->header.stamp).seconds());

    rclcpp::Time msg_time = msg->header.stamp;

    publish_topics(msg_time);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageGrabber>();

    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}