/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
 *
 */

#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber() = default;

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    explicit ImageGrabber(ImuGrabber *pImuGb) : mpImuGb(pImuGb) {}

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void GrabImageCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("orb_slam3");
    rclcpp::Logger logger = node->get_logger();
    clock_ = node->get_clock();
    RCLCPP_INFO(logger, "orb_slam3 started");

    if (argc > 1)
    {
        RCLCPP_WARN(logger, "Arguments supplied via command line are ignored.");
    }

    std::string node_name = node->get_name();
    image_transport::ImageTransport image_transport(node);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    std::string imu_topic, image_topic, image_topic_compressed;
    node->declare_parameter<std::string>("imu_topic", "/imu/data");
    node->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    node->declare_parameter<std::string>("image_topic_compressed", "/camera/color/image_raw/compressed");
    node->get_parameter("imu_topic", imu_topic);
    node->get_parameter("image_topic", image_topic);
    node->get_parameter("image_topic_compressed", image_topic_compressed);
    // imu_topic = "/imu/data";
    // image_topic = "/camera/color/image_raw";
    // imu_topic = "/imu0";
    // image_topic = "/cam0/image_raw";
    // image_topic_compressed = "/camera/color/image_raw/compressed";
    // default_voc_file = "/home/eric/ws_orb3_ros2/src/ORB_SLAM3_ROS2/orb_slam3/Vocabulary/ORBvoc.txt";

    std::string voc_file, settings_file;
    node->declare_parameter<std::string>("voc_file", default_voc_file);
    node->declare_parameter<std::string>("settings_file", std::string(PROJECT_SOURCE_DIR) +
                                                                  "/config/Monocular-Inertial/usb_cam_bno055.yaml");
    node->get_parameter("voc_file", voc_file);
    node->get_parameter("settings_file", settings_file);
    RCLCPP_INFO(logger, "voc_file: %s, settings_file: %s", voc_file.c_str(), settings_file.c_str());

    node->declare_parameter<std::string>("world_frame_id", "map");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->get_parameter("world_frame_id", world_frame_id);
    node->get_parameter("cam_frame_id", cam_frame_id);

    bool enable_pangolin{};
    node->declare_parameter<bool>("enable_pangolin", true);
    node->get_parameter("enable_pangolin", enable_pangolin);

    bool use_compressed{};
    node->declare_parameter<bool>("use_compressed", true);
    node->get_parameter("use_compressed", use_compressed);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 1000, std::bind(&ImuGrabber::GrabImu, &imugb, std::placeholders::_1));

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img_compressed;
    if (use_compressed)
        sub_img_compressed = node->create_subscription<sensor_msgs::msg::CompressedImage>(
                image_topic_compressed, 100,
                std::bind(&ImageGrabber::GrabImageCompressed, &igb, std::placeholders::_1));
    else
        sub_img = node->create_subscription<sensor_msgs::msg::Image>(
                image_topic, 100, std::bind(&ImageGrabber::GrabImage, &igb, std::placeholders::_1));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Subscribed to " << image_topic_compressed << ", use_compressed: " << use_compressed);

    // Assuming setup_publishers and setup_services are defined functions
    setup_publishers(node, image_transport, node_name);
    setup_services(node, node_name);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    rclcpp::spin(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////
void ImageGrabber::GrabImageCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg)
{
    RCLCPP_WARN_ONCE(rclcpp::get_logger("rclcpp"), "GrabImageCompressed");
    // msg -> sensor_msgs::Image &img_msg
    //  Convert the compressed image to an OpenCV Mat
    cv::Mat compressed_image = cv::imdecode(cv::Mat(compressed_msg->data), cv::IMREAD_COLOR);
    // cv::imshow("compressed_image", compressed_image);
    // cv::waitKey(1);

    if (compressed_image.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to decode compressed image");
        return;
    }

    // Convert the OpenCV Mat to a ROS sensor_msgs::Image
    std_msgs::msg::Header header = compressed_msg->header; // Use the same header as the input message
    cv_bridge::CvImage cv_image(header, "bgr8", compressed_image);

    GrabImage(cv_image.toImageMsg());
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return {};
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    auto start_time = clock_->now();
    while (rclcpp::ok())
    {
        if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
        {
            cv::Mat im;
            double tIm = 0;

            tIm = toSec(img0Buf.front()->header.stamp);
            // cout << "stamp = " << tIm << ", stamp.sec = "<< img0Buf.front()->header.stamp.sec << ", nanosec = "<<
            // img0Buf.front()->header.stamp.nanosec <<endl;
            if (tIm > toSec(mpImuGb->imuBuf.back()->header.stamp))
                continue;

            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            rclcpp::Time msg_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() && toSec(mpImuGb->imuBuf.front()->header.stamp) <= tIm)
                {
                    double t = toSec(mpImuGb->imuBuf.front()->header.stamp);

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y,
                            mpImuGb->imuBuf.front()->angular_velocity.z;

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // auto dt = clock_->now() - start_time;
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dt = %f, HZ = %f, vImuMeas: %d", dt.seconds(), 1.0 /
            // dt.seconds(), vImuMeas.size()); start_time = clock_->now();

            // ORB-SLAM3 runs in TrackMonocular()
            /*Sophus::SE3f Tcw = */ pSLAM->TrackMonocular(im, tIm, vImuMeas);

            publish_topics(msg_time, Wbb);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"), "GrabImu");
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}
