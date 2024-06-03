/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc and ros_rgbd.cc
 *
 */

#include "common.h"


using namespace std;

class ImuGrabber
{
public:
    ImuGrabber()= default;
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    explicit ImageGrabber(ImuGrabber *pImuGb) : mpImuGb(pImuGb) {}

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::SharedPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("orb_slam3");
    image_transport::ImageTransport image_transport(node);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    clock_ = node->get_clock();

    std::string voc_file, settings_file;
    node->declare_parameter<std::string>("voc_file", default_voc_file);
    node->declare_parameter<std::string>("settings_file", std::string(PROJECT_SOURCE_DIR) + "/config/RGB-D-Inertial/RealSense_D435i.yaml");
    node->get_parameter("voc_file", voc_file);
    node->get_parameter("settings_file", settings_file);

    bool enable_pangolin{};
    node->declare_parameter<bool>("enable_pangolin", true);
    node->get_parameter("enable_pangolin", enable_pangolin);

    std::string world_frame_id, cam_frame_id, imu_frame_id;
    node->declare_parameter<std::string>("world_frame_id", "map");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->declare_parameter<std::string>("imu_frame_id", "imu");
    node->get_parameter("world_frame_id", world_frame_id);
    node->get_parameter("cam_frame_id", cam_frame_id);
    node->get_parameter("imu_frame_id", imu_frame_id);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_RGBD;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    std::string rgb_img_topic = "/camera/rgb/image_raw";
    std::string depth_img_topic = "/camera/depth_registered/image_raw";
    std::string imu_topic = "/imu/data";

    //debug code
    // rgb_img_topic = "/cam0/image_raw";
    // depth_img_topic = "/cam1/image_raw";
    // imu_topic = "/imu0";

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 1000, std::bind(&ImuGrabber::GrabImu, &imugb, std::placeholders::_1));

    auto sub_rgb_img =
            std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, rgb_img_topic);
    auto sub_depth_img = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            node, depth_img_topic);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
            approximate_sync_policy;
    auto syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
            approximate_sync_policy(10), *sub_rgb_img, *sub_depth_img);
    syncApproximate->registerCallback(&ImageGrabber::GrabRGBD, &igb);

    // Assuming setup_publishers and setup_services are adapted for ROS2 as well
    setup_publishers(node, image_transport, node->get_name());
    setup_services(node, node->get_name());

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

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                            const sensor_msgs::msg::Image::SharedPtr msgD)
{
    std::lock_guard<std::mutex> lock(mBufMutex);

    if (!imgRGBBuf.empty())
        imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);

    if (!imgDBuf.empty())
        imgDBuf.pop();
    imgDBuf.push(msgD);
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    while (rclcpp::ok())
    {
        if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            cv::Mat im, depth;
            double tIm = 0;

            tIm = toSec(imgRGBBuf.front()->header.stamp);
            if (tIm > toSec(mpImuGb->imuBuf.back()->header.stamp))
                continue;

            this->mBufMutex.lock();
            rclcpp::Time msg_time = imgRGBBuf.front()->header.stamp;
            im = GetImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            depth = GetImage(imgDBuf.front());
            imgDBuf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            vImuMeas.clear();
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
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

            // ORB-SLAM3 runs in TrackRGBD()
            /*Sophus::SE3f Tcw = */pSLAM->TrackRGBD(im, depth, tIm, vImuMeas);

            publish_topics(msg_time, Wbb);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    imuBuf.push(imu_msg);
}
