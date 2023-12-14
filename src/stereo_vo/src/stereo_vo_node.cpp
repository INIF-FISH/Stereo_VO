#include "../include/stereo_vo_node.h"

using namespace std::placeholders;

namespace stereo_vo
{
    StereoVONode::StereoVONode(const rclcpp::NodeOptions &options)
        : Node("stereo_vo", options)
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("stereo_cam_usb");

        rclcpp::QoS qos(1);
        qos.reliability();
        qos.keep_last(1);

        rmw_qos_profile_t rmw_qos = qos.get_rmw_qos_profile();
        this->camera_params_path = this->declare_parameter<std::string>("camera_params_path", package_share_directory + "/params/camera.yaml");
        this->_detect_thread = std::make_shared<std::thread>(std::thread(std::bind(&StereoVONode::detectCallback, this)));
        this->_image_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "stereo_cam_usb/image_raw",
                                                                                                              std::bind(&StereoVONode::imageCallback, this, _1), "raw", rmw_qos));
        this->_image_pub = std::make_shared<image_transport::Publisher>(image_transport::create_publisher(this, "/stereo_vo/pub_image_raw", rmw_qos));
        RCLCPP_INFO(this->get_logger(), "Register callback for stereo_usb camera ...");

        this->_StereoVO = std::make_shared<StereoVO>(StereoVO());
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
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid image with incorrect frame_id.");
        }
    }

    void StereoVONode::detectCallback()
    {
        while (this->_is_alive)
        {
            cv::Mat right_frame, left_frame;
            std::unique_lock<std::shared_timed_mutex> ulk(this->image_deque_mutex);
            if (this->image_deque_.empty())
                continue;
            auto img_info = this->image_deque_.front();
            this->image_deque_.pop_front();
            ulk.unlock();
            cv::Mat image_full = cv_bridge::toCvCopy(img_info, img_info->encoding)->image;
            cv::Mat image_left = image_full(cv::Rect(0, 0, img_info->width / 2, img_info->height));
            cv::Mat image_right = image_full(cv::Rect(img_info->width / 2, 0, img_info->width / 2, img_info->height));
            if (!this->_StereoVO->_is_params_inited())
            {
                if (!this->_StereoVO->read_param(this->camera_params_path, img_info->width, img_info->height))
                {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Failed to read camera params !");
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "Success to read camera params !");
            }
            if (!this->_StereoVO->_is_calced_Transform())
            {
                this->_StereoVO->calcTransform();
                this->_StereoVO->init();
                RCLCPP_INFO(this->get_logger(), "Success to init !");
            }
            this->_StereoVO->addFrame(image_left, image_right);
        }
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