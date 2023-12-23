#ifndef _STEREO_VO_NODE_H
#define _STEREO_VO_NODE_H

#include "./stereo_vo/stereo_vo.h"

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h" 

namespace stereo_vo
{
    using namespace std::chrono;
    class StereoVONode : public rclcpp::Node
    {
    private:
        std::shared_ptr<StereoVO> _StereoVO;
        std::string camera_params_path;

    public:
        StereoVONode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~StereoVONode();

    private:
        std::shared_ptr<image_transport::Subscriber> _image_sub;
        std::shared_ptr<image_transport::Publisher> _image_pub;
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_pub;

    private:
        std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_deque_;
        std::shared_timed_mutex image_deque_mutex;

        bool _is_alive = true;

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        void detectCallback();

    public:
        std::shared_ptr<std::thread> _detect_thread;

    };
} // namespace stereo_vo

#endif //_STEREO_VO_NODE_H