#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//#include <opencv/cvwimage.h>
#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#include "cv_functions.h"

// INclude msg class
//#include "rsd_camera_functions/Num.h"
#include <rsd_camera_functions/line_points.h>

//#include "std_msgs/String.h"
//#include "std_msgs/float32.h"

#include <sstream>

using namespace cv;

// Define names of subscriber and publisher
//#define SUBSCRIBER "rsd_camera/bar_camera"
#define SUBSCRIBER "/usb_cam/image_raw"
//#define SUBSCRIBER "rsd_camera/image"

//#define PUBLISHER "camera/angle"
#define PUBLISHER "rsd_camera/line_points"

class SubscribeAndPublish
{
	public:
	SubscribeAndPublish()
	{
		//Topic to publish
		pub_ = n_.advertise<rsd_camera_functions::line_points>(PUBLISHER, 1);
		
		//Topic to subscribe
		sub_ = n_.subscribe(SUBSCRIBER, 1, &SubscribeAndPublish::callback, this);
	}

	// Define Callback function called by the subscriber
	void callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// To select wheter to show or not the output image
		bool show_image=FALSE;
		
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

		// Start the line detection
		
		// Resize the image
		 Mat image;
	   	// resize(cv_ptr->image,image,cvSize(cv_ptr->image.size[1]/4,cv_ptr->image.size[0]/4));
		 image=cv_ptr->image;

	   	 Mat image_hsv;
	    	 cvtColor(image,image_hsv,CV_RGB2HSV );	

		 // Filter by color
		 Mat mask;
		 inRange(image_hsv,Scalar(100,50,50), Scalar(130, 255, 255),mask);

		// Open to delete small areas 
		 Mat mask2;
		 morphologyEx(mask,mask2,cv::MORPH_OPEN,getStructuringElement(MORPH_RECT, Size(10,10)));
		
		 // Detect lines
		 vector<Vec2f> lines;

		 // Find lines of preprocesed image
		 HoughLines( mask2, lines, 1, CV_PI/360, 350);

		// Merge lines
		vector<Vec2f> lines_merged=merge_lines(lines);
	
		// Show result image
/*
		if (show_image)	
		{
			//imshow("Color segmented",mask);
			//imshow("Opened mask",mask2);
			display_lines(image,lines_merged,"Lines merged");
			//display_lines(image,lines,"Lines");	
		}
*/
		cv::waitKey(3);
	
		vector<Vec2f> result=transform_point(lines_merged,image.cols,image.rows);

		// Print the points of the lines
	/*
		for (int i=0;i<result.size();i++)
		{
			cout<<"Point "<<i<<": "<<result[i]<<endl;	
		}
	*/
		rsd_camera_functions::line_points to_publish;
		if(result.size()!=0)
		{		
			to_publish.x1=result[0][0];
			to_publish.y1=result[0][1];
			to_publish.x2=result[1][0];
			to_publish.y2=result[1][1];	
			pub_.publish(to_publish);

		}		

		// Publish the string
		//pub_.publish(str);
		
	}

	private:
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Subscriber sub_;

};


//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "rsd_line_detector");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
