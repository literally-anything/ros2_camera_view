#include "camera_view/camera_view.hpp"


namespace camera_view
{
    CameraViewNode::CameraViewNode(const rclcpp::NodeOptions &options) :
            Node("camera_view", options)
    {
    }

    void CameraViewNode::release()
    {
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;
    std::shared_ptr<camera_view::CameraViewNode> node = std::make_shared<camera_view::CameraViewNode>(options);

    rclcpp::spin(node);

    node->release();

    rclcpp::shutdown();
    return 0;
}
