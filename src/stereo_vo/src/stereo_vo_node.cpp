#include "../include/stereo_vo_node.h"

using namespace std::placeholders;

namespace stereo_vo
{
    StereoVONode::StereoVONode(const rclcpp::NodeOptions &options)
        : Node("stereo_dso", options)
    {
        rclcpp::QoS qos(1);
        qos.reliability();
        qos.keep_last(1);

        rmw_qos_profile_t rmw_qos = qos.get_rmw_qos_profile();
        this->_detect_timer = rclcpp::create_timer(this, this->get_clock(), 30ms, std::bind(&StereoVONode::detectCallback, this));
        this->_image_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/stereo_usb/image_raw",
                                                                                                              std::bind(&StereoVONode::imageCallback, this, _1), "raw", rmw_qos));
        this->_image_pub = std::make_shared<image_transport::Publisher>(image_transport::create_publisher(this, "/stereo_dso/pub_image_raw", rmw_qos));
        RCLCPP_INFO(this->get_logger(), "Registered callback for stereo_usb camera ...");

        // if(!read_param(C_P,"/home/nine-fish/projects/StereoDSO/Stereo_DSO_ws/src/stereo_dso/params/camera.yaml"))
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to read params for stereo_usb camera ...");
        //     abort();
        // }
        // else
        //     RCLCPP_INFO(this->get_logger(), "Success to read params for stereo_usb camera ...");
    }

    StereoVONode::~StereoVONode()
    {
    }

    void StereoVONode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        std::string frame_id = img_info->header.frame_id;
        if (frame_id == "usb_stereo_frame")
        {
            std::unique_lock<std::shared_timed_mutex> ulk(this->image_deque_mutex);
            this->image_deque_.emplace_back(img_info);
            while (this->image_deque_.size() > 10)
                this->image_deque_.pop_front();
            ulk.unlock();
            RCLCPP_INFO(this->get_logger(), "Received image from stereo_usb.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid image with incorrect frame_id.");
        }
    }

    void StereoVONode::detectCallback()
    {
        cv::Mat right_frame, left_frame;
        std::unique_lock<std::shared_timed_mutex> ulk(this->image_deque_mutex);
        if (this->image_deque_.empty())
            return;
        auto img_info = this->image_deque_.front();
        this->image_deque_.pop_front();
        ulk.unlock();
        RCLCPP_INFO(this->get_logger(), "Process image from stereo_usb.");
        cv::Mat image_full = cv_bridge::toCvCopy(img_info, img_info->encoding)->image.clone();
        cv::cvtColor(image_full,image_full,cv::COLOR_RGB2GRAY);
        cv::Mat image_right = image_full(cv::Rect(0, 0, img_info->width / 2, img_info->height));
        cv::Mat image_left = image_full(cv::Rect(img_info->width / 2, 0, img_info->width / 2, img_info->height));

        auto image_ros = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_left).toImageMsg();
        this->_image_pub->publish(std::move(image_ros));
        cv::waitKey(1);
    }

} // namespace stereo_dso

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stereo_vo::StereoVONode>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_vo::StereoVONode)