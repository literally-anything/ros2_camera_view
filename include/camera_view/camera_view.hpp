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
    };
}

#endif //CAMERA_VIEW_CAMERA_VIEW_HPP
