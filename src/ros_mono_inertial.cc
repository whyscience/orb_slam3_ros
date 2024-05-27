/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
 *
 */

#include "common.h"

using namespace std;

rclcpp::Clock::SharedPtr clock_;
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

    std::string imu_topic = "/imu";
    std::string image_topic = "/camera/image_raw";

    std::string voc_file, settings_file;
    node->declare_parameter<std::string>("voc_file", default_voc_file);
    node->declare_parameter<std::string>("settings_file",
                                         std::string(PROJECT_SOURCE_DIR) + "/config/Monocular-Inertial/EuRoC.yaml");
    node->get_parameter("voc_file", voc_file);
    node->get_parameter("settings_file", settings_file);
    RCLCPP_INFO(logger, "voc_file: %s, settings_file: %s", voc_file.c_str(), settings_file.c_str());

    // debug code
    imu_topic = "/imu0";
    image_topic = "/cam0/image_raw";

    node->declare_parameter<std::string>("world_frame_id", "map");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->get_parameter("world_frame_id", world_frame_id);
    node->get_parameter("cam_frame_id", cam_frame_id);

    bool enable_pangolin{};
    node->declare_parameter<bool>("enable_pangolin", true);
    node->get_parameter("enable_pangolin", enable_pangolin);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 1000, std::bind(&ImuGrabber::GrabImu, &imugb, std::placeholders::_1));
    auto sub_img = node->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 100, std::bind(&ImageGrabber::GrabImage, &igb, std::placeholders::_1));

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
            //cout << "stamp = " << tIm << ", stamp.sec = "<< img0Buf.front()->header.stamp.sec << ", nanosec = "<< img0Buf.front()->header.stamp.nanosec <<endl;
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

            //auto dt = clock_->now() - start_time;
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dt = %f, HZ = %f, vImuMeas: %d", dt.seconds(), 1.0 / dt.seconds(), vImuMeas.size());
            //start_time = clock_->now();

            // ORB-SLAM3 runs in TrackMonocular()
            /*Sophus::SE3f Tcw = */ pSLAM->TrackMonocular(im, tIm, vImuMeas);

            publish_topics(msg_time, Wbb);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}
