#ifndef CAMERA_VIEW_CAMERA_VIEW_HPP
#define CAMERA_VIEW_CAMERA_VIEW_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

namespace camera_view
{
    class CameraViewNode : public rclcpp::Node
    {
    public:
        explicit CameraViewNode(const rclcpp::NodeOptions &);

        void release();

    private:
        std::string windowTitle;
        std::string encoding;
        int framerateLimit;

        image_transport::Subscriber imageSubscriber;
        cv::Mat image;

        void initParameters();


        static rcl_interfaces::msg::ParameterDescriptor_<std::allocator<void>> generateParamDescriptor(
                std::string description
        );

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    };
}

#endif //CAMERA_VIEW_CAMERA_VIEW_HPP
