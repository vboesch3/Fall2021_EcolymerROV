#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <
static const std::string OPENCV_WINDOW = "Window";
class Follower
{
   ros::NodeHandle n;
   image_transport::ImageTransport it__;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
public:
   Follower():it_(n)
    {
     image_sub = it_.subscribe("/camera/image_raw", 1,
	&Follower::imageCb,this);
     image_pub = it_.advertise("/follower/output_video",1);
   
   cv::namedWindow(OPENCV_WINDOW);
    }
   ~Follower()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
	{
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      catch(cv_bridge::Exceptions& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

    }
};
        
int main(int argc, char **argv)
{
   ros::init(argc, argv,"follower");
   Follower follower;
   ros::spin();
   return 0; 
}

