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
#include "videoray/Throttle.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sstream>
#include "videoray/Status.h"
#include "videoray/UHRIComm.h"

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
videoray::Throttle throttle_;
//VideoRayComm comm;
float err;
float erry;
float previous_err = 0;
float integral = 0;
float derivative = 0;
float output;
float Kp=0.01;
float Ki=0;
float Kd=0.01;
float des_speed=0;

static const std::string OPENCV_WINDOW = "Image window";
int lowH_low = 0; // 13
int highH_low = 10; // 65 Set Hue
int lowH_high = 160; // 13
int highH_high = 180;

int lowS = 70; //72
int highS = 255; // 192Set Saturation

int lowV = 50; // 193
int highV = 255; //255 Set Value
int imageWidth;
int imageHeight;
int search_top;
int search_bottom;

int cx;
int cy;
  cv::Mat diffPts;
  cv::Mat ptsx, ptsy;
  cv::Mat canny_output;
  cv::Mat imgOriginal; //Input image
  cv::Mat hsvImg; //HSV Image
  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat threshImg; //Threshimage
  cv::Mat M;
 cv::Mat drawing;
 std::vector< cv::Vec4i > linesP;
  std::vector< std::vector<cv::Point> > contours;
std::vector< std::vector<cv::Point> > biggest_contours;
cv::Scalar color;
cv::RNG rng(12345);
 double dist;
 double dist_new;
size_t largest_contours = 0;
float heading_port, heading_star, speed_port, speed_star, speed_vert; //speed_vert is a new addition
ros::Publisher throttle_pub;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
     ros::Publisher videoray_status_pub_; 
public:
  ImageConverter()
    : it_(nh_)
  {

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/videoray/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    throttle_pub = nh_.advertise<videoray::Throttle>("/videoray/throttle", 1);
    //throttle_pub = nh_.advertise<videoray::Throttle>("/rqt_thrust_monitor/throttle_cmd", 1);
  }

  ~ImageConverter()
  {
     throttle_.PortInput =  0;
     throttle_.StarInput =  0;
     throttle_.VertInput = 0; 
     throttle_pub.publish(throttle_); 

    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
	    
cv::cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);

cv::inRange( hsvImg, cv::Scalar(lowH_low, lowS, lowV), cv::Scalar(highH_low, highS, highV), mask1);
cv::inRange( hsvImg, cv::Scalar(lowH_high, lowS, lowV), cv::Scalar(highH_high, highS, highV), mask2);
threshImg = mask1 | mask2;
 imageWidth = imgOriginal.size().width;
 imageHeight = imgOriginal.size().height;
std::cout << "width" << imageWidth << std::endl;
std::cout << "height" << imageHeight << std::endl;
 search_top = 3 * imageHeight / 4;
 search_bottom = search_top + 20;
for (int i; i < imageHeight; i++){
for(int j; j < imageWidth; j++){
if ( i < search_top && j < imageWidth){
	hsvImg.at<uchar>(i, j, 0) = 0 ;

} 
if (   search_top < i && j > imageWidth){
	hsvImg.at<uchar>(i, j, 0) = 0 ;
}}
}
cv::GaussianBlur(threshImg, threshImg, cv::Size(3,3), 0); //reduce details
cv::dilate(threshImg, threshImg, 0); //removes noise
cv::erode(threshImg, threshImg, 0); //trims image

    
    cv::Canny( threshImg, canny_output, 100, 100*2, 3 ); //edge detection
 
    cv::findContours( canny_output, contours, cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE ); //finds edge 
    std::vector<cv::Moments> mu(contours.size() ); //calculates moments
    for( int i = 0; i < contours.size(); i++ )
    {
        mu[i] = cv::moments( contours[i] );
    }
    std::vector<cv::Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
        //add 1e-5 to avoid division by zero
        mc[i] = cv::Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
                         static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) ); //x center, y center
        //std::cout << "mc[" << i << "]=" << mc[i] << std::endl;
    }

    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
	if (contours.size() > 1 ){	
	dist =	cv::arcLength(contours[0], true);
		largest_contours = 0;    
	for( size_t i = 0; i < contours.size(); i++ ){
		dist_new = cv::arcLength(contours[i], true);
		//std::cout << "old dist: " << dist << " new dist: " << dist_new << std::endl;
		if ( dist_new > dist) {
			dist = dist_new;
			largest_contours = i;
			//std::cout<< "largest countour: " << i << std::endl;


}
}
   
        color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::drawContours( imgOriginal, contours, (int)largest_contours, color, 2 );//highlight the color
        cv::circle( imgOriginal, mc[largest_contours], 4, color, -1 );
		
	err = mc[largest_contours].x - imageWidth/2;
	erry = mc[largest_contours].y - imageHeight/2; //added
	speed_port=(err)/imageWidth*30+15;
	speed_star=-(err)/imageWidth*30+15;
	speed_vert=(erry)/imageHeight*40+15; //added
	
	
	
	std::cout<< "speed star " << speed_star << std::endl;
	std::cout<< "speed port " << speed_port << std::endl;
	std::cout<< "speed vert " << speed_vert << std::endl; //added
	//port = left, star = right, vert = up & down
	
	//if(abs(err)<20){
	  //speed_port = (double)VideoRay_Speed;
	  //speed_star = (double)VideoRay_Speed;
	//}
	//else if(err < -290){
	//sharpturn left
      	  //speed_port = -1 * (double)VideoRay_Speed;
	  //speed_star = (double)VideoRay_Speed;
	//}
	//else if(err<0){
	  //slight turn left
	  //speed_port = 0;
	  //speed_star = (double)VideoRay_Speed;	
	//}
	//else if ( err > 290){
	  //sharp turn right
          //speed_port = (double)VideoRay_Speed;
	  //speed_star = -1 * (double)VideoRay_Speed;
	//}
	//else if(err>0){
	  //slight turn right
          //speed_port = (double)VideoRay_Speed;
	  //speed_star = 0;	
	//}
	
  }   

cv::namedWindow("imgOriginal", CV_WINDOW_NORMAL);
cv::namedWindow("threshImg", CV_WINDOW_NORMAL);

     /* Create trackbars in "threshImg" window to adjust according to object and environment.*/
  
  cv::createTrackbar("LowH", "threshImg", &lowH_low, 180); //Hue (0 - 179)
  cv::createTrackbar("HighH", "threshImg", &highH_low, 180);

  cv::createTrackbar("LowS", "threshImg", &lowS, 255); //Saturation (0 - 255)
  cv::createTrackbar("HighS", "threshImg", &highS, 255);

  cv::createTrackbar("LowV", "threshImg", &lowV, 255); //Value (0 - 255)
  cv::createTrackbar("HighV", "threshImg", &highV, 255);

  cv::createTrackbar("Speed", "threshImg", &VideoRay_Speed, 100);
  

  cv::imshow("imgOriginal", imgOriginal);     // show windows
  cv::imshow("threshImg", threshImg);


    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
     image_pub_.publish(cv_ptr->toImageMsg());
     //VideoRayComm comm;
     //comm.set_vertical_thruster(speed_vert);
     throttle_.PortInput=speed_port;
     throttle_.StarInput=speed_star;
     throttle_.VertInput= speed_vert; //was originally set to 0
     throttle_pub.publish(throttle_);        

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
