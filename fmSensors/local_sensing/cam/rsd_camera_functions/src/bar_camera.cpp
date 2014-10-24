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
#define PUBLISHER "rsd_camera/bar_camera"
//#define SUBSCRIBER "rsd_camera/image"

//#define SUBSCRIBER  "kinect_test/image"
//#define PUBLISHER "kinect_test/image"

using namespace cv;


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "rsd_camera_publisher");
   	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise(PUBLISHER, 1);

	


	CvCapture* capture;
	Mat frame;

	capture = cvCaptureFromCAM( -1 );

	 if( capture )
   	{
	    
	   frame = cvQueryFrame( capture );
	   imshow("Capture try", frame);
	  
	}else
	{
		ROS_ERROR("Not capture"); 
		return 0;   	
	}

	

	ros::Rate loop_rate(5);
	
	while (nh.ok()) 
	{

	     cv_bridge::CvImage cvi;
	     cvi.header.stamp = ros::Time::now();
	     cvi.header.frame_id = "image";
	     cvi.encoding = "bgr8";
	
             frame = cvQueryFrame( capture );
	/*	
	     imshow("Capture try",frame);
	     waitKey(3);
	*/
	     cvi.image=frame;


	     sensor_msgs::Image im;
	     cvi.toImageMsg(im);

	     pub.publish(im);
	     ros::spinOnce();

	     loop_rate.sleep();
	}
	
}


