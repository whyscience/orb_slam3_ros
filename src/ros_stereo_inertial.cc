/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo_inertial.cc
 *
 */

#include "common.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber()= default;

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    explicit ImageGrabber(ImuGrabber *pImuGb) : mpImuGb(pImuGb) {}

    void GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;
    ImuGrabber *mpImuGb;
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

    std::string world_frame_id, cam_frame_id, imu_frame_id;
    node->declare_parameter<std::string>("world_frame_id", "world");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->declare_parameter<std::string>("imu_frame_id", "imu");
    node->get_parameter("world_frame_id", world_frame_id);
    node->get_parameter("cam_frame_id", cam_frame_id);
    node->get_parameter("imu_frame_id", imu_frame_id);

    bool enable_pangolin{};
    node->declare_parameter<bool>("enable_pangolin", true);
    node->get_parameter("enable_pangolin", enable_pangolin);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_STEREO;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 1000, std::bind(&ImuGrabber::GrabImu, &imugb, std::placeholders::_1));
    auto sub_img_left = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/left/image_raw", 100, std::bind(&ImageGrabber::GrabImageLeft, &igb, std::placeholders::_1));
    auto sub_img_right = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/right/image_raw", 100, std::bind(&ImageGrabber::GrabImageRight, &igb, std::placeholders::_1));

    setup_publishers(node, image_transport, node_name);
    setup_services(node, node_name);

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

void ImageGrabber::GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutexLeft);
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
}

void ImageGrabber::GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutexRight);
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
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
    const double maxTimeDiff = 0.01;
    while (rclcpp::ok())
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            tImLeft = toSec(imgLeftBuf.front()->header.stamp);
            tImRight = toSec(imgRightBuf.front()->header.stamp);

            {
                std::lock_guard<std::mutex> lock(mBufMutexRight);
                while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
                {
                    imgRightBuf.pop();
                    tImRight = toSec(imgRightBuf.front()->header.stamp);
                }
            }

            {
                std::lock_guard<std::mutex> lock(mBufMutexLeft);
                while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
                {
                    imgLeftBuf.pop();
                    tImLeft = toSec(imgLeftBuf.front()->header.stamp);
                }
            }

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > toSec(mpImuGb->imuBuf.back()->header.stamp))
                continue;

            {
                std::lock_guard<std::mutex> lock(mBufMutexLeft);
                imLeft = GetImage(imgLeftBuf.front());
                imgLeftBuf.pop();
            }

            {
                std::lock_guard<std::mutex> lock(mBufMutexRight);
                imRight = GetImage(imgRightBuf.front());
                imgRightBuf.pop();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            {
                std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
                if (!mpImuGb->imuBuf.empty())
                {
                    // Load imu measurements from buffer
                    vImuMeas.clear();
                    while (!mpImuGb->imuBuf.empty() && toSec(mpImuGb->imuBuf.front()->header.stamp) <= tImLeft)
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
            }

            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f Tcw = pSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            publish_topics(rclcpp::Time(imgLeftBuf.front()->header.stamp), Wbb);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
}
