//ROS + OpenCv
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sstream>
#include <list>
#include <vector>
#include <cmath>

// Videoray
#include "videoray/Status.h"
#include "videoray/UHRIComm.h"
#include "videoray/Throttle.h"

// darknet_ros_msgs
#include "videoray/BoundingBox.h"
#include "videoray/BoundingBoxes.h"

double depth_ref = 0;
double speed_ref = 0;
double heading_ref = 0;
int VideoRay_Speed = 0;
double sharpturn_speed = 0;
double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;
double depth_err = 0;
double speed_err = 0;
double heading_err = 0;
double heading = 0;
double heading_weight = 0.5;
double speed_weight = 0.5;
double K_heading = 1;
double K_speed = 10;
double K_depth = 50;

float previous_err = 0;
float integral = 0;
float derivative = 0;
float output;
float Kp=0.01;
float Ki=0;
float Kd=0.01;
float des_speed=0;

static const std::string OPENCV_WINDOW = "Image window";
int imageWidth = 720;
int imageHeight = 480;
int maxa;
int xPosCenter;
int  yPosCenter;
bool detectedPlastic=false;
float errx;
float erry;
float prevx;
float prevy;
int width;
std::list<double> area;
cv::Mat imgOriginal; //Input image
cv::RNG rng(12345);
float heading_port, heading_star, speed_port, speed_star, speed_vert; //speed_vert is a new addition

videoray::Throttle throttle_;
ros::Subscriber boxes;
ros::Publisher throttle_pub;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  //Subscribe
  //Publishers
  //ros::Publisher videoray_status_pub_; 
  
public:
  ImageConverter(): it_(nh_)
  {

    // Subscribe to input video feed and publish output video feed
    boxes = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::boxesCallback,this);
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,&ImageConverter::image, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    throttle_pub = nh_.advertise<videoray::Throttle>("/videoray/throttle", 1);
  }

  ~ImageConverter()
  {
    throttle_.PortInput =  0;
    throttle_.StarInput =  0;
    throttle_.VertInput = 0; 
    throttle_pub.publish(throttle_); 
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void boxesCallback(const videoray::BoundingBoxes& msg){
    std::vector<videoray::BoundingBox> vec = msg.bounding_boxes;
    double maxa=0;
    int index=0;
    errx=0;
    erry=0;
    prevx=0;
    prevy=0;
    xPosCenter=0;
    yPosCenter=0;
    width=0;
    //std::cout<< "ENTER BOX CALLBACK" << speed_star << std::endl;
    for(int i =0; i < vec.size(); i++){
      if (msg.bounding_boxes[i].Class == "plastic") {
        detectedPlastic = true;
      }
      else {
        throttle_.PortInput =  0;
        throttle_.StarInput =  0;
        throttle_.VertInput = 0; 
        throttle_pub.publish(throttle_);
        detectedPlastic= false;
      }

      if(detectedPlastic == true){
          double a = (msg.bounding_boxes[i].xmax-msg.bounding_boxes[i].xmin) * (msg.bounding_boxes[i].ymax-msg.bounding_boxes[i].ymin);
          area.push_back(a);
          if(a > maxa){
            maxa=a;
            index=i;
       }
      }
    }
    xPosCenter = msg.bounding_boxes[index].xmin + (msg.bounding_boxes[index].xmax - msg.bounding_boxes[index].xmin) * 0.5;
    yPosCenter = msg.bounding_boxes[index].ymin + (msg.bounding_boxes[index].ymax - msg.bounding_boxes[index].ymin) * 0.5;
    
    width=msg.bounding_boxes[index].xmax-msg.bounding_boxes[index].xmin;
    if(width>450){
	throttle_.PortInput =  0;
        throttle_.StarInput =  0;
        throttle_.VertInput = 0; 
        throttle_pub.publish(throttle_);
    }
    else{
	    std::cout<< "width " << width << std::endl;
	    errx = xPosCenter - imageWidth/2;
	    erry = yPosCenter - imageHeight/2; //added
	    prevx=errx;
	    prevy=erry;
	    if(errx<0){
	    	speed_port=(errx)/imageWidth*40-15*abs((errx-prevx)/imageWidth);
	    	speed_star=-(errx)/imageWidth*40+15*abs((errx-prevx)/imageWidth);
	    	speed_vert=(erry)/imageHeight*40+15*abs((erry-prevy)/imageHeight);
	    }
	    else{
	    	speed_port=(errx)/imageWidth*40+15*abs((errx-prevx)/imageWidth);
	    	speed_star=-(errx)/imageWidth*40-15*abs((errx-prevx)/imageWidth);
	    	speed_vert=(erry)/imageHeight*40+15*abs((erry-prevy)/imageHeight);
	    }
	    std::cout<< "errx " << errx << "prevx" << prevx<< std::endl;
	    std::cout<< "speed star " << speed_star << std::endl;
	    std::cout<< "speed port " << speed_port << std::endl;
	    std::cout<< "speed vert " << speed_vert << std::endl; //added

	    throttle_.PortInput=speed_port;
	    throttle_.StarInput=speed_star;
	    throttle_.VertInput= speed_vert; //was originally set to 0
	    throttle_pub.publish(throttle_);
    }
  }

  void image(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    imgOriginal = cv_ptr->image;
    cv::circle( imgOriginal, cv::Point(xPosCenter,yPosCenter), 4, cv::Scalar(0,0,255), -1 );
    cv::namedWindow("imgOriginal", CV_WINDOW_NORMAL);
    cv::imshow("imgOriginal", imgOriginal);   
    cv::waitKey(3);
  }

};

void mySigintHandler(int sig){
 throttle_.PortInput =  0;
 throttle_.StarInput =  0;
 throttle_.VertInput = 0; 
 throttle_pub.publish(throttle_);
 ros::shutdown();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  signal(SIGINT, mySigintHandler);
  ros::spin();     
return 0;
}
