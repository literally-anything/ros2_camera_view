#include "camera_view/camera_view.hpp"


namespace camera_view
{
    CameraViewNode::CameraViewNode(const rclcpp::NodeOptions &options) :
            Node("camera_view", options)
    {
        initParameters();

        windowTitle = get_fully_qualified_name();

        std::string transport = get_parameter("transport").as_string();
        encoding = get_parameter("encoding").as_string();
        framerateLimit = (int) get_parameter("framerate_limit").as_int();
        RCLCPP_INFO(this->get_logger(), "Initializing camera_view node with transport %s and encoding %s",
                    transport.c_str(), encoding.c_str());

        imageSubscriber = image_transport::create_subscription(this, "my_camera",
                                                               [this](auto &msg)
                                                               {
                                                                   imageCallback(msg);
                                                               },
                                                               transport);
    }

    void CameraViewNode::release()
    {
        imageSubscriber.shutdown();
        capture.release();
    }

    void CameraViewNode::initParameters()
    {
        declare_parameter("transport", "raw", generateParamDescriptor(
                "The image transport to use (raw, compressed, theora)"
        ));
        declare_parameter("encoding", "bgr8", generateParamDescriptor(
                "The image encoding to use (bgr8, mono8, bgra8, rgb8, rgba8, mono16)"
        ));
        declare_parameter("framerate_limit", -1, generateParamDescriptor(
                "If there are more frames in a second than this, they are skipped"
        ));
    }


    rcl_interfaces::msg::ParameterDescriptor_<std::allocator<void>> CameraViewNode::generateParamDescriptor(
            std::string description
    )
    {
        rcl_interfaces::msg::ParameterDescriptor_<std::allocator<void>> paramDescriptor =
                rcl_interfaces::msg::ParameterDescriptor{};
        paramDescriptor.description = std::move(description);
        paramDescriptor.read_only = true;

        return paramDescriptor;
    }

    void CameraViewNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        image = cv_bridge::toCvShare(msg, encoding)->image;

        cv::imshow(windowTitle, image);
        cv::waitKey(1);
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
