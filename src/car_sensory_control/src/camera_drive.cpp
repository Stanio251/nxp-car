//============================================================================
// Name        : camera_drive.cpp
// Author      : Zenab Sadi (zenab.sadi@tum.de)
// Version     : March 2025
//============================================================================
#include <memory>
#include <chrono> //time library
#include <functional> // provides a set of predefined templates for a set of objects- use dby ros2
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "image_transport/image_transport.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <math.h>
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;
using namespace cv;
using namespace std::chrono_literals;



class CameraAnalyser : public rclcpp::Node
{
  public:
    CameraAnalyser()
    : Node("camera_drive"),cmnd_vec({0.1, 0.0, 0.0, 0.0, 0.0, 0.0}),prev_lane_lines(),prev_right_line(Vec4i()),prev_left_line(Vec4i()), count_(0), pos_error(0), height(256), width(256)
    {
        
      image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
      "/images_rgbd/image", 10, std::bind(&CameraAnalyser::EdgeDetection_callback, this, _1));
      //image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
      //"/camera/image_raw", 10, std::bind(&CameraAnalyser::EdgeDetection_callback, this, _1));
      //image_transport::TransportHints hints(this,"compressed");
      //image_subscriber = image_transport::create_subscription(this,
      //"/camera/image_raw", 
      //std::bind(&CameraAnalyser::EdgeDetection_callback, this, _1), &hints);

      publisherObject=this->create_publisher<geometry_msgs::msg::Twist>("controller/cmd_vel", 10); //Topic name MUST be ACCURATE , msgs buffer size
      pubProcessedImages=this->create_publisher<sensor_msgs::msg::Image>("ProcessedImage",10);
      pubAllDetectedLines=this->create_publisher<sensor_msgs::msg::Image>("DetectedLines",10);
      pubError=this->create_publisher<std_msgs::msg::Float32>("PositionError",10);
      
      
    }

  private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_intrinsic_param;
    std::array<double,9UL> intrinsic_param;
    std::vector<Vec4i> lane_lines;
    std::vector<double> cmnd_vec; 
    std::vector<Vec4i> prev_lane_lines;//history
    Vec4i right_line;
    Vec4i left_line;
    Vec4i prev_right_line;
    Vec4i prev_left_line;
    double m_right,m_left;
    
    rclcpp::TimerBase::SharedPtr timerObject;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherObject;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubProcessedImages;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubAllDetectedLines;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubError;
    size_t count_;
    _Float32 pos_error;
    int height;
    int width;
    
    void SendSteeringCommands(){
      // we recieve an image with 256x256 pixels
      int img_center_x=128; //width
      int img_center_y=200; //height 
      std::size_t lane_lines_size=lane_lines.size();
      int dist_deltax_right,dist_deltax_left;

      auto msg = geometry_msgs::msg::Twist();
      //move forward carfully- if you have no info
      msg.linear.x = cmnd_vec[0];  // Forward velocity in m/s
      msg.linear.y = cmnd_vec[1];  // Sideways velocity in m/s
      msg.linear.z = cmnd_vec[2];  // Upward velocity in m/s
      msg.angular.x = cmnd_vec[3]; // Roll rotation in rad/s
      msg.angular.y = cmnd_vec[4]; // Pitch rotation in rad/s
      msg.angular.z = cmnd_vec[5];  // Yaw rotation in rad/s

      if (lane_lines_size==2){
        //calculate the distnace from teh right , left lines 
        //center_x-line_x
        dist_deltax_right=Delta_x(img_center_x,img_center_y,right_line);
        //dist_deltax_right=Euc_Distance((right_line[0]+right_line[2])/2,(right_line[1]+right_line[3])/2,img_center_x,img_center_y);
        dist_deltax_left=Delta_x(img_center_x,img_center_y,left_line);
        RCLCPP_INFO(this->get_logger(), "distance= %d",dist_deltax_left-dist_deltax_right); // first it was 58
        RCLCPP_INFO(this->get_logger(), "distance to the left= %d, distance to the right line = %d",dist_deltax_left, dist_deltax_right);

        RCLCPP_INFO(this->get_logger(), "Right line p1= (%d,%d)  p2=(%d,%d)",right_line[0],right_line[1],right_line[2], right_line[3]);
        RCLCPP_INFO(this->get_logger(), "Left line p1= (%d,%d)  p2=(%d,%d)",left_line[0],left_line[1],left_line[2], left_line[3]);
        if (dist_deltax_right>0){
          //move to the left
          cmnd_vec[5]=0.2;
          cmnd_vec[0]=0.2;
          RCLCPP_INFO(this->get_logger(), "move to the left"); 
        }
        else{
          if(dist_deltax_left<0){
            //move to the right
            cmnd_vec[5]=-0.2;
            cmnd_vec[0]=0.2; 
            RCLCPP_INFO(this->get_logger(), "move to the right"); 
          }
          else
          {
            if(abs(dist_deltax_left-abs(dist_deltax_right))>10) //in pixels //tolerate 10 pixels 
            {
              if (dist_deltax_left>abs(dist_deltax_right)){
                //move to the left -- get closer to the left line
                cmnd_vec[5]=0.2;
                cmnd_vec[0]=0.2;
                RCLCPP_INFO(this->get_logger(), "move to the a little to the left"); 
              }
              else{
                //move to the right --- get closer to the righ line
                cmnd_vec[5]=-0.2;
                cmnd_vec[0]=0.2;
                RCLCPP_INFO(this->get_logger(), "move to the a little to the right"); 
              }
            }
            else{
              //move forward 
              cmnd_vec[5]=0.0;
              cmnd_vec[0]=0.3;
            }
          }

        }

        
      }else{
        if (lane_lines_size==1){
          if (right_line == cv::Vec4i()){
            //double dl=Delta_x(img_center_x,img_center_y,left_line);
            // if (dl<20){
            //move to the right 
            cmnd_vec[5]=-0.5;
            cmnd_vec[0]=0.2;
            RCLCPP_INFO(this->get_logger(), "Move to the Right");
            // }
            // else {
            //   cmnd_vec[5]=-0.3;
            //   cmnd_vec[0]=0.2;
            // }

          }
          else{
            if (left_line == cv::Vec4i()){
            
            //double dr=Delta_x(img_center_x,img_center_y,right_line);
            //if (dr<20){
            //move to the left
            cmnd_vec[5]=0.5;
            cmnd_vec[0]=0.2;
            RCLCPP_INFO(this->get_logger(), "Move to the Left");
            //}
            // else {
            //   cmnd_vec[5]=0.3;
            //   cmnd_vec[0]=0.2;
            // }
          }
        }

        }
        else{ //no lines detected (no even in the prev step)
          RCLCPP_INFO(this->get_logger(), "No lines detected");
        }
      }

      msg.angular.z = cmnd_vec[5];
      msg.linear.x = cmnd_vec[0];
      

      RCLCPP_INFO(this->get_logger(), "Publishing Twist: linear(%f, %f, %f), angular(%f, %f, %f)",
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z);

      // Publish the message to the "cmd_vel" topic
      //publisherObject->publish(msg);

    }

    void EdgeDetection_callback(const sensor_msgs::msg::Image & msg)
    {
      RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------------------");
      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //cv::Mat image = cv_ptr->image;
      cv_bridge::CvImagePtr cv_ptr;
      Mat src, dst;
      int lowThreshold = 0;
      //const int max_lowThreshold = 100;
      const int ratio = 3;
      const int kernel_size = 3;
      //const char* window_name = "Edge Map";
      try{
        cv_ptr=cv_bridge::toCvCopy (msg,"rgb8"); 	

      }catch(cv_bridge::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      //get rid of the upper half of the image 
      //RCLCPP_INFO(this->get_logger(), "The height of the image: %d",msg.height);

      Mat gray_image;
      cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

      //filter noise using gaussian filter
      Mat blurred_image;
      GaussianBlur(gray_image, blurred_image, Size(3, 3), 1.5); // 	double 	sigmaY = 0, int  borderType = BORDER_DEFAULT

      Mat colorfiltered=ColorFilter(blurred_image);

      //Trianglar Mask for the img before applying line detection algorithms
      // imshow("the img after color filtering ", colorfiltered);
      // // waitKey(0);

      // std::vector<cv::Point> vertices_ROI= TriangularROI(colorfiltered);
      // blurred_image= region_of_interest(blurred_image, vertices_ROI);
      // colorfiltered= region_of_interest(colorfiltered, vertices_ROI);

      // imshow("the img after color filtering ", colorfiltered);
      // waitKey(0);

      Mat detected_edges;
      Canny( blurred_image, detected_edges, 100, lowThreshold*ratio, kernel_size );



      // std::string image_file="canny_detector"+ std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "The counetrs value = %ld",++count_);

      //RCLCPP_INFO(this->get_logger(), "The height of the canny image: %d and the width of the canny image is:%d",detected_edges.rows, detected_edges.cols);

      //get rid of the upper half of the image (the height of the image is 256)-> will get rid 0f 150, canny detector preserve the original dimensions : 256x256
      int start_height=0;
      int end_height=200;

      for (int row = start_height; row < end_height; ++row) {
        for (int col = 0; col < detected_edges.cols; ++col) {
            uchar &pixel = detected_edges.at<uchar>(row, col);  // For a 3-channel (BGR) image
            
            // Modify pixel: make the pixel black  
            pixel= 0;
        }
    }
      cv::Mat combined_detection;
      cv::bitwise_or(detected_edges,colorfiltered,combined_detection);

      std::vector<cv::Point> vertices_ROI= TriangularROI(combined_detection);
      combined_detection= region_of_interest(combined_detection, vertices_ROI);
      // imshow("ROI combined_detection ", combined_detection);
      // waitKey(0);

      LineDetection(combined_detection,msg); //here we use Hough transformation

    }
    cv::Mat ColorFilter (cv::Mat gray_img)
    {
      ///gray scale image - gray_img
      cv::Mat binary;
      cv::threshold(gray_img, binary, 50, 255, cv::THRESH_BINARY_INV);
      return binary;
    }

    void LineDetection(const Mat & detected_edges, const sensor_msgs::msg::Image & msg) //houghtrafo
    {

      //Probabilistic
      // Probabilistic Line Transform
      std::vector<Vec4i> linesP; // will hold the results of the detection
      Mat img_prob_lines;
      cvtColor(detected_edges, img_prob_lines, COLOR_GRAY2BGR);
      HoughLinesP(detected_edges, linesP, 1, CV_PI/180, 15, 15, 10 ); // runs the actual detection
      // Draw the lines
      RCLCPP_INFO(this->get_logger(), "number of Lines: %ld", linesP.size());
      //double r=0;
      for( size_t i = 0; i < linesP.size(); i++ )
      {
          Vec4i l = linesP[i];
          line( img_prob_lines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,width), 3, LINE_AA);
      }
      line( img_prob_lines, Point(0, width/2), Point(width, width/2), Scalar(0,0,width), 3, LINE_AA);
      line( img_prob_lines, Point(width/2, width), Point(width/2, 0), Scalar(0,0,width), 3, LINE_AA);

      // imshow(" ", img_prob_lines);
      // waitKey(0);


      //-------------------------------------------------------------------------------------
      cv_bridge::CvImage lines_msg;
      lines_msg.header= msg.header;
      lines_msg.encoding="bgr8";
      lines_msg.image=img_prob_lines;
      sensor_msgs::msg::Image ros_lines_image;
      lines_msg.toImageMsg(ros_lines_image);
      pubAllDetectedLines->publish(ros_lines_image);
      //-------------------------------------------------------------------------------------
      //StreetLines(linesP);
      ModStreetLines(linesP);
      
      Mat img_final_filtered_lines;
      cvtColor(detected_edges, img_final_filtered_lines, COLOR_GRAY2BGR);
      
      for( size_t i = 0; i < lane_lines.size(); i++ )
      {
        Vec4i l1 = lane_lines[i];
        line( img_final_filtered_lines, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(0,0,255), 3, LINE_AA);
      }

      //imshow( "Final Hough Line transform", img_final_filtered_lines );

      cv_bridge::CvImage out_msg;
      out_msg.header= msg.header;
      out_msg.encoding="bgr8";
      out_msg.image=img_final_filtered_lines;
      sensor_msgs::msg::Image ros_image;
      out_msg.toImageMsg(ros_image);
      pubProcessedImages->publish(ros_image);

      SendPositionError();

    }

    void StreetLines(std::vector<Vec4i>linesP){
      std::vector<Vec4i> img_lines;

      double x, m, s=0;
      int size_linesP=linesP.size();
      double min_dist_right=0,min_dist_left=0,dist;

      for (int i=0;i<size_linesP;i++){
        if (linesP[i][0]!=linesP[i][3]){
          m=CalcSlope(linesP[i]);
          RCLCPP_INFO(this->get_logger(), "the slope is= %f", m);



          if (abs(m)<0.001) {
            dist=0;  
          } 
          else {
            x=CalcXlineEquation(255, linesP[i][0],linesP[i][1], m);
            s=128-x;
          }
        }
        else {
          x=linesP[i][0];
          s=128-x;
          RCLCPP_INFO(this->get_logger(), "straight Line shouldnt Make a Problem !!!");
        }

        dist=Distance(linesP[i][0],linesP[i][1],linesP[i][2],linesP[i][3],127.5,5);
        
        if ((s>0 && min_dist_left==0)|| (s>0 && dist<min_dist_left))
        {
          min_dist_left=dist;
          left_line=linesP[i];
        }
        if ((s<0 && min_dist_right==0)||(s<0 && abs(dist)<min_dist_right))
        {
          min_dist_right=abs(dist);
          right_line=linesP[i];
        }
        
        }
      

      if (min_dist_left!=0){
        img_lines.push_back(left_line);
        RCLCPP_INFO(this->get_logger(), "min_dist_left= %f", min_dist_left);
      }
      if (min_dist_right!=0){
        img_lines.push_back(right_line);
        RCLCPP_INFO(this->get_logger(), "min_dist_right= %f", min_dist_right);
        
      }
      lane_lines=img_lines; // what gets pushed first will be in [0]
      //Filter out Intersection Lines 
      if (lane_lines.size()>1)
        FilterImgLines();
      
    }

    void ModStreetLines(std::vector<Vec4i>linesP){
      std::vector<Vec4i> img_lines;

      double x, s=0,m;
      int size_linesP=linesP.size(), py=1,px=1, ply_max=-1, plx_max=-1, pry_max=-1, prx_max=-1; 
      //py=the index of the point we are interested in, on line i (which point has max y).  
      //px=the index of the point we are interested in, on line i (which point has max x). **px and py can indicate two different points 
      double dist;

      for (int i=0;i<size_linesP;i++){
        if (linesP[i][1]>=linesP[i][3]){
          x=linesP[i][0];
          py=1;
          if (linesP[i][0]!=linesP[i][3]){
            m=CalcSlope(linesP[i]);
          }
          if (m!=0){
            x=CalcXlineEquation(255, linesP[i][0],linesP[i][1], m);
          }
  
        }
        else{
          x=linesP[i][2];
          py=2;
          if (linesP[i][0]!=linesP[i][3]){
            m=CalcSlope(linesP[i]);
          }
          if (m!=0){
            x=CalcXlineEquation(255, linesP[i][0],linesP[i][1], m);
          }

        }
        if (linesP[i][0]>=linesP[i][2]){
          px=0;
        }
        else{
          px=2;
        }
        s=128-x;

        dist=Euc_Distance(linesP[i][py-1],linesP[i][py],127.5,256);
        
        if (s>0) 
        {
          if (ply_max==-1){
            ply_max=linesP[i][py];
            plx_max<linesP[i][px];
            left_line=linesP[i];
          }
          else{
            if (ply_max<linesP[i][py] && plx_max<linesP[i][px]){
              ply_max=linesP[i][py];
              plx_max=linesP[i][px];
              left_line=linesP[i];

            }
          }
          
        }
        if (s<0) 
        {
          if (pry_max==-1){
            pry_max=linesP[i][py];
            prx_max=linesP[i][px];
            right_line=linesP[i];
          }
          else{
            if (pry_max<linesP[i][py] && prx_max<linesP[i][px]){
              pry_max=linesP[i][py];
              prx_max=linesP[i][px];
              right_line=linesP[i];

            }
          }
          
        }
        
      }
      

      if (ply_max!=-1){
        img_lines.push_back(left_line);
      }
      if (pry_max!=-1){
        img_lines.push_back(right_line);
        
      }
      lane_lines=img_lines; // what gets pushed first will be in [0]
      //Filter out Intersection Lines 
      // if (lane_lines.size()>1)
      //   FilterImgLines();
      
    }

    //----------------------------------------------------------------------------------------
    //---------------------------------------------------------------------------------------------------------------
    double CalcSlope(Vec4i line1){
      //RCLCPP_INFO(this->get_logger(), "in CalcSteepness");

      double m1;
      int x11=line1[0],y11=line1[1],x12=line1[2],y12=line1[3];
      m1=double(y12-y11)/(x12-x11);
      return m1;

    }
    
    double CalcXlineEquation(int y, int x0, int y0, double m) //calculate x from line equation
    {
      return (y-y0)/m+x0;
    }

    double Distance(int x1, int y1, int x2, int y2, double x3, double y3){
      //calculate the distance of a point to a line
      if ((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1)==0){
        RCLCPP_INFO(this->get_logger(), "!! Two Identical Points Cant make a Line !!!");
      }
      double dot_prod=(x3-x1)*(x2-x1)+(y3-y1)*(y2-y1); //,<p3,p2>=|p3||p2|cos(alpha)
      dot_prod=dot_prod/std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1)); // proj=|p3|cos(alpha)
      double d=std::sqrt( ((y3 - y1) * (y3 - y1) + (x3 - x1) * (x3 - x1))- (dot_prod*dot_prod));//p3^2-proj^2=d^2
      return d;
    }
    double Delta_x(int center_x, int center_y, Vec4i points_on_line){
      if (points_on_line[0]==points_on_line[2]){
        return center_x-points_on_line[0];
      }
      else{
        double m=CalcSlope(points_on_line);
        double x=CalcXlineEquation(center_y, points_on_line[0],points_on_line[1], m);
        return center_x-x;

      }
    }

    double Intersection_point(int x11,int y11,int x21,int y21, double m1, double m2){

      double y12,x12;
      x12=(m1*x11-m2*x21+y21-y11)/(m1-m2);
      y12=m1*(x12-x11)+y11;
      return y12;

    }
    void FilterImgLines()
    {
      //Search for intersection lines and filters them 
      //Uses lane lines after the applying street lines filtering 
      int x11,y11,x21,y21;
      double m1,m2;
      Vec4i l1=lane_lines[0];
      Vec4i l2=lane_lines[1];
      m1=CalcSlope(l1);
      m2=CalcSlope(l2);
      if (m1!=m2){
        x11=l1[0];
        y11=l1[1];
        x21=l2[0];
        y21=l2[1];
        double y12=Intersection_point(x11,y11,x21,y21,m1,m2);
        int end_height=200;
        if (y12>=end_height){
          //delete the further line
          std::vector<cv::Vec4i> filteredLines;
          double r1=Distance(l1[0], l1[1],l1[2], l1[3],128,128); // this is not used for any imp calculatoin
          double r2=Distance(l2[0], l2[1],l2[2], l2[3],128,128); // this is not used for any imp calculatoin
          if (r1>r2){
            filteredLines.push_back(l2);
          }
          else {
            filteredLines.push_back(l1);
          }
          lane_lines=filteredLines;
        }
      }

    }

    double TwoPointsDist(Vec4i l1, double mx, double my)
    {
      // calculate the distance from the car to the points making the lines
      double d1 = sqrt(pow(mx-l1[0],2)+pow(my-l1[1],2));
      double d2 = sqrt(pow(mx-l1[2],2)+pow(my-l1[3],2));

      return min({d1,d2});
      
    }

    Mat region_of_interest(const Mat& img, const std::vector<cv::Point>& vertices) {
      // Create a black mask with the same size as the input image
      Mat mask = cv::Mat::zeros(img.size(), img.type());
  
      // Define the color to fill the ROI (255 for white in a single-channel mask)
      Scalar match_mask_color = (img.channels() == 1) ? cv::Scalar(255) : cv::Scalar(255, 255, 255);
  
      // Fill the polygonal ROI in the mask
      std::vector<std::vector<cv::Point>> pts = {vertices}; 
      cv::fillPoly(mask, pts, match_mask_color);
  
      // Apply the mask to the image
      Mat masked_image;
      cv::bitwise_and(img, mask, masked_image);
  
      return masked_image;
  }

    std::vector<cv::Point> TriangularROI(Mat img){
      height = img.rows;
      width = img.cols;
      RCLCPP_INFO(this->get_logger(), "height = %d, width =%d", height, width);

      std::vector<cv::Point> region_of_interest_vertices = {
        cv::Point(0,249),// Bottom-left( if I want to consider the lowest point in the pic height)
        cv::Point(0, round(0.6*height)),              
        cv::Point(width / 2, 0.3*height),  // Middle-top
        cv::Point(width, round(0.6*height)),
        cv::Point(width,249)           // Bottom-right
      };
      
      return region_of_interest_vertices;

    }

    // //check if the two lines lay on the same curve
    // void f(){
    //   Point p1,p2;
    //   std::vector<std::vector<cv::Point>> v(2, std::vector<cv::Point>(256)); //mx2 m=256
    //   double d=Euc_Distance(p1[0],p1[1],p2[0],p2[1]);
    //   if(d<5){
    //     //on the same curve
    //     v[i][j]=p1;
    //   }
    // }

    double Euc_Distance (double x1, double y1,double x2, double y2){
      double d=sqrt(pow(x2-x1,2)+pow(y2-y1,2));
      return d;
    }

    void SendPositionError(){
      // we recieve an image with 256x256 pixels
      int img_center_x=128; //width
      int img_center_y=128; //height 
      std::size_t lane_lines_size=lane_lines.size();
      int dist_deltax_right,dist_deltax_left;

      auto pos_error_msg = std_msgs::msg::Float32();
      

      if (lane_lines_size==2){
        //calculate the distnace from teh right , left lines 
        //center_x-line_x
        dist_deltax_right=Delta_x(img_center_x,img_center_y,right_line);
        dist_deltax_left=Delta_x(img_center_x,img_center_y,left_line);
        RCLCPP_INFO(this->get_logger(), "distance= %d",dist_deltax_left-dist_deltax_right); // first it was 58
        RCLCPP_INFO(this->get_logger(), "distance to the left= %d, distance to the right line = %d",dist_deltax_left, dist_deltax_right);

        RCLCPP_INFO(this->get_logger(), "Right line p1= (%d,%d)  p2=(%d,%d)",right_line[0],right_line[1],right_line[2], right_line[3]);
        RCLCPP_INFO(this->get_logger(), "Left line p1= (%d,%d)  p2=(%d,%d)",left_line[0],left_line[1],left_line[2], left_line[3]);

        pos_error= dist_deltax_right+dist_deltax_left;

        
      }else{
        if (lane_lines_size==1){
          if (right_line == cv::Vec4i()){
            RCLCPP_INFO(this->get_logger(), "Move to the Right");
            dist_deltax_left=Delta_x(img_center_x,img_center_y,left_line);
            pos_error= dist_deltax_left-width;
            
          
          }
          else{
            if (left_line == cv::Vec4i()){
            
            RCLCPP_INFO(this->get_logger(), "Move to the Left");
            dist_deltax_right=Delta_x(img_center_x,img_center_y,right_line);
            pos_error= dist_deltax_right+width;
          }
        }

        }
        else{ //no lines detected (no even in the prev step)
          RCLCPP_INFO(this->get_logger(), "No lines detected");
        }
      }

      RCLCPP_INFO(this->get_logger(), "Publishing Position Error %f",pos_error);
      if (pos_error>1000){
        pos_error=1000;
      }
      pos_error_msg.data=pos_error;



      // Publish the message to the "cmd_vel" topic
      pubError->publish(pos_error_msg);

    }

    
    //----------------------------------------------------------------------------------------------------------------

};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraAnalyser>()); //creates a new instance of MinimalSubscriber and returns a shared_ptr that manages its lifetime. The functionality of this statement is entirely about memory management and object creation.
    rclcpp::shutdown();
    return 0;
}
