#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class Controller : public rclcpp::Node {
public:
  Controller() : Node("my_node") {
    RCLCPP_INFO(this->get_logger(), "Hello from controller node!");
    
    //define perspective transormation points (source, destination)
    src_pts_ = {
      cv::Point2f(30, 20),   // top-left
      cv::Point2f(145, 20),  // top-right
      cv::Point2f(176, 144), // bottom-right
      cv::Point2f(0, 144)   // bottom-left
    };

    dst_pts_ = {
      cv::Point2f(30, 20),     // top-left
      cv::Point2f(145, 20),    // top-right
      cv::Point2f(121, 144),  // bottom-right
      cv::Point2f(55, 144)    // bottom-left
    };

    //compute PerspectiveTransform matrix
    M_ = cv::getPerspectiveTransform(src_pts_, dst_pts_);

    // Create a subscription to /video topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/video",                   // topic name
      10,                         // queue size
      std::bind(&Controller::image_callback, this, std::placeholders::_1)
    );

    //create birdview publisher
    bird_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/bird_view", 10);
  }

private:
  // Callback function when a new image arrives
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image: %ux%u", msg->width, msg->height);
    try {
      cv::Mat image = cv_bridge::toCvCopy(msg, "mono8")->image;
      cv::bitwise_not(image, image);
      // transform to birds eye view
      cv::Mat bird_eye;
      cv::warpPerspective(image, bird_eye, M_, image.size());
      // convert image to binary
      cv::threshold(bird_eye, bird_eye, 240, 255, cv::THRESH_BINARY);

      // draw perspective points in image matrix
      for (const auto& pt : src_pts_) {
        cv::circle(image, pt, 3, cv::Scalar(255), -1);  // white circle, radius 3
      }

      //create bird_view message and publish it
      auto msg_out = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "mono8",  // or "bgr8" if you use color
        bird_eye
      ).toImageMsg();
        
      msg_out->header = msg->header; // optionally copy timestamp
      bird_pub_->publish(*msg_out);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

  }
  //Variable declaration
  std::vector<cv::Point2f> src_pts_;
  std::vector<cv::Point2f> dst_pts_;
  cv::Mat M_; 
  cv::Mat binary; 
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bird_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
