#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher()
    : Node("video_publisher")
    {
        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video", 10);

        // Open camera
        cap_.open(3);  // Change camera index if needed
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video device");
        } else {
            RCLCPP_INFO(this->get_logger(), "Video device opened successfully");
        }

        // Timer for 15 FPS
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 15),
            std::bind(&VideoPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_.read(frame)) {
            // Convert to grayscale
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // Convert to ROS image message
            std_msgs::msg::Header header;
            header.stamp = this->now();
            auto msg = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();

            // Publish
            publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read from camera");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
