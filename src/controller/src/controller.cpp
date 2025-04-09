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
      cv::Point2f(144, 20),  // top-right
      cv::Point2f(176, 144), // bottom-right
      cv::Point2f(0, 144)   // bottom-left
    };

    dst_pts_ = {
      cv::Point2f(30, 20),     // top-left
      cv::Point2f(145, 20),    // top-right
      cv::Point2f(121, 144),  // bottom-right
      cv::Point2f(55, 144)    // bottom-left
    };

    //create empty warp mask
    std::vector<cv::Point> dst_polygon = {
      cv::Point2f(0, 16), cv::Point2f(176, 16), cv::Point2f(118, 144), cv::Point2f(58, 144)
    };
    //fill in warp mask
    cv::fillConvexPoly(warp_mask, dst_polygon, 255);
    cv::imshow("Warp Mask", warp_mask);

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
    // RCLCPP_INFO(this->get_logger(), "Received image: %ux%u", msg->width, msg->height);
    try {
      //read image data and invert the colors
      cv::Mat image = cv_bridge::toCvCopy(msg, "mono8")->image;
      cv::bitwise_not(image, image);
      // transform to birds eye view
      cv::Mat bird_eye;
      cv::warpPerspective(image, bird_eye, M_, image.size());

      // // convert image to binary
      // // Step 1: Apply Gaussian blur (optional, helps reduce noise)
      // cv::Mat blurred;
      // cv::GaussianBlur(bird_eye, blurred, cv::Size(3, 3), 0);

      // // Step 2: Use Sobel or Canny for edge detection
      // cv::Mat edges;
      // cv::Canny(blurred, edges, 50, 150);  // you can tune thresholds

      //Apply Sobel in x-direction
      cv::Mat grad_x;
      cv::Sobel(bird_eye, grad_x, CV_32F, 1, 0, 3);  // 32F so we get negative values too
      
      //
      cv::Mat borders = cv::Mat::zeros(grad_x.size(), CV_8U);

      int mid_col = grad_x.cols / 2;
      
      // Get left and right halves of grad_x
      cv::Mat left_half = grad_x(cv::Range::all(), cv::Range(0, mid_col));
      cv::Mat right_half = grad_x(cv::Range::all(), cv::Range(mid_col, grad_x.cols));
      
      // Get references to left and right halves of result
      cv::Mat left_result = borders(cv::Range::all(), cv::Range(0, mid_col));
      cv::Mat right_result = borders(cv::Range::all(), cv::Range(mid_col, grad_x.cols));
      
      // Set left half: negative values -> 1
      left_result.setTo(255, left_half < -100); // for 32F we get -1024, 1024 values, threshhold for removing noise from track plane
      // Set right half: positive values -> 1
      right_result.setTo(255, right_half > 100);
      
      cv::imshow("Borders", borders);

      // //visualisation of sobel matrix
      // cv::Mat grad_x_vis;
      // cv::convertScaleAbs(grad_x, grad_x_vis);  // absolute value & convert to 8-bit
      // cv::imshow("grad_x (abs)", grad_x_vis);

      // cv::threshold(bird_eye, bird_eye, 240, 255, cv::THRESH_BINARY);

      // cv::Mat masked = cv::Mat::zeros(grad_x.size(), CV_32F);
      // grad_x.copyTo(masked, warp_mask);  // keeps values where mask != 0
      // cv::convertScaleAbs(masked, grad_x_vis);  // absolute value & convert to 8-bit
      // cv::imshow("grad x masked", grad_x_vis);


      //calculate histogram for bottom half for the masked sobel edge detection matrix
      cv::Mat bottom_half = borders(cv::Range(borders.rows / 3, borders.rows), cv::Range::all());  // adjust to borders.rows/2 for half image
      cv::Mat histogram;
      cv::reduce(bottom_half, histogram, 0, cv::REDUCE_SUM, CV_32S);

      // Convert histogram to vector for easier handling
      std::vector<int> hist_vector(histogram.cols);
      for (int i = 0; i < histogram.cols; ++i) {
        hist_vector[i] = histogram.at<int>(0, i);
      }

      // Find left and right peaks
      int midpoint = hist_vector.size() / 2;
      int left_base = std::distance(hist_vector.begin(), std::max_element(hist_vector.begin(), hist_vector.begin() + midpoint));
      int right_base = std::distance(hist_vector.begin(), std::max_element(hist_vector.begin() + midpoint, hist_vector.end()));
      RCLCPP_INFO(this->get_logger(), "left x right: %ix%i", left_base, right_base);

      // Visualize histogram (optional)
      cv::Mat hist_image(100, bird_eye.cols, CV_8UC1, cv::Scalar(0));
      for (int i = 0; i < hist_vector.size(); i++) {
        int val = std::min(hist_vector[i] / 100, 100);  // scale it down to fit
        cv::line(hist_image, cv::Point(i, 99), cv::Point(i, 99 - val), cv::Scalar(255));
      }

      cv::imshow("Histogram", hist_image);
      cv::imshow("Bird Eye", bird_eye);
      cv::waitKey(1);

      // draw perspective points in image matrix
      for (const auto& pt : src_pts_) {
        cv::circle(image, pt, 3, cv::Scalar(255), -1);  // white circle, radius 3
      }

      //Parameters
      // Sliding windows calculation
      int y = borders.rows;
      std::vector<int> lx, rx;

      //set sliding windows dimension
      int window_height = 10;
      int window_width = 30;
      cv::Mat msk = borders.clone();
      cv::cvtColor(msk, msk, cv::COLOR_GRAY2BGR);  // for colored rectangles

      while (y > 0) {
          // --- LEFT WINDOW ---
          int x_left_start = std::max(left_base - window_width / 2, 0);
          int x_left_end = std::min(left_base + window_width / 2, borders.cols);
          int y_start = std::max(y - window_height, 0);

          cv::Mat left_window = borders(cv::Range(y_start, y), cv::Range(x_left_start, x_left_end));
          std::vector<std::vector<cv::Point>> contours_left;
          cv::findContours(left_window.clone(), contours_left, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          for (const auto& contour : contours_left) {
              cv::Moments M = cv::moments(contour);
              if (M.m00 != 0) {
                  int cx = static_cast<int>(M.m10 / M.m00);
                  int cy = static_cast<int>(M.m01 / M.m00);
                  int abs_x = x_left_start + cx;
                  lx.push_back(abs_x);
                  left_base = abs_x;  // update base for next window
              }
          }

          // --- RIGHT WINDOW ---
          int x_right_start = std::max(right_base - window_width / 2, 0);
          int x_right_end = std::min(right_base + window_width / 2, borders.cols);

          cv::Mat right_window = borders(cv::Range(y_start, y), cv::Range(x_right_start, x_right_end));
          std::vector<std::vector<cv::Point>> contours_right;
          cv::findContours(right_window.clone(), contours_right, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          for (const auto& contour : contours_right) {
              cv::Moments M = cv::moments(contour);
              if (M.m00 != 0) {
                  int cx = static_cast<int>(M.m10 / M.m00);
                  int cy = static_cast<int>(M.m01 / M.m00);
                  int abs_x = x_right_start + cx;
                  rx.push_back(abs_x);
                  right_base = abs_x;  // update base for next window
              }
          }

          // --- Visualize sliding windows ---
          cv::rectangle(msk, cv::Point(x_left_start, y), cv::Point(x_left_end, y_start), cv::Scalar(0, 255, 0), 1);
          cv::rectangle(msk, cv::Point(x_right_start, y), cv::Point(x_right_end, y_start), cv::Scalar(255, 0, 0), 1);

          y -= window_height;
      }

      cv::imshow("Sliding Windows", msk);


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
  cv::Mat warp_mask = cv::Mat::zeros(144, 176, CV_8UC1);  //adjust resolution here

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
