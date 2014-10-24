#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>


//Find the correct name for the kinect topic
#define PUBLISHER "rsd_camera/image"
#define SUBSCRIBER "rsd_camera/image"

//#define SUBSCRIBER  "kinect_test/image"
//#define PUBLISHER "kinect_test/image"

using namespace cv;


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "rsd_image_publisher");
   	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise(PUBLISHER, 1);

	

	cv::Mat imageA = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test1.jpg");
	cv::Mat imageB = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test2.jpg");
	cv::Mat imageC = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test3.jpg");
	cv::Mat imageD = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test4.jpg");
	cv::Mat imageE = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test5.jpg");
	cv::Mat imageF = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test6.jpg");
	cv::Mat imageG = cv::imread("/home/maik/catkin_ws/src/rsd_camera_functions/src/line_test/test7.jpg");

	if(imageA.empty() or imageB.empty() or imageC.empty() or imageD.empty()){
		ROS_ERROR("Some image not loaded"); 
		return 0;   
	}
	
	

	ros::Rate loop_rate(1);
	int count=0;
	int n_images=7;
	while (nh.ok()) 
	{

	     cv_bridge::CvImage cvi;
	     cvi.header.stamp = ros::Time::now();
	     cvi.header.frame_id = "image";
	     cvi.encoding = "bgr8";

	     if (count%n_images==0){cvi.image = imageA;}
	     if (count%n_images==1){cvi.image = imageB;}
	     if (count%n_images==2){cvi.image = imageC;}
	     if (count%n_images==3){cvi.image = imageD;}
		if (count%n_images==4){cvi.image = imageE;}
		if (count%n_images==5){cvi.image = imageF;}
		if (count%n_images==6){cvi.image = imageG;}




	     count++;
	     sensor_msgs::Image im;
	     cvi.toImageMsg(im);


	     pub.publish(im);
	     ros::spinOnce();

	     loop_rate.sleep();
	}
	
}


