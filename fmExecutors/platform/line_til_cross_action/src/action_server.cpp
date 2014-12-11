#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <line_til_cross_action/GocellAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <msgs/IntStamped.h>

#define IMAGE_SUBSCRIBER "usb_cam/image_raw"
#define IMAGE_PUBLISHER "line_action/image_raw"
#define PID_ENABLE_PUBLISHER "line_action/pid_enable"
#define CROSS_DETECTED_SUBSCRIBER "line_node/cross_detected"
#define BAR_CODE_SUBSCRIBER "line_node/barcode"
#define ENCODER_LEFT_SUBSCRIBER "fmInformation/encoder_left"


class GocellAction
{
	protected:
		ros::NodeHandle nh_;
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		actionlib::SimpleActionServer<line_til_cross_action::GocellAction> as_;
		std::string action_name_;
		// create messages that are used to published feedback/result
		line_til_cross_action::GocellFeedback feedback_;
		line_til_cross_action::GocellResult result_;
		line_til_cross_action::GocellGoal goal_;
		ros::Publisher image_pub_;
		ros::Publisher pid_pub_;
		ros::Subscriber image_sub_;
		ros::Subscriber cross_sub_;
		ros::Subscriber barcode_sub_;
        ros::Subscriber encoder_left_sub_;
		cv::Mat image_;
		std::string barcode_value_;
		bool success_;
		int counter_;
        int encoder_val_;


	public:
		GocellAction(std::string name) :
		// action server
		as_(nh_, name, boost::bind(&GocellAction::executeCB, this, _1), false),
		action_name_(name)
		{
			// Sibscribers and publishers
			image_pub_ = nh_.advertise<sensor_msgs::Image>(IMAGE_PUBLISHER, 1);
			image_sub_ = nh_.subscribe(IMAGE_SUBSCRIBER, 1, &GocellAction::callback_image, this);
			pid_pub_=nh_.advertise<std_msgs::Bool>(PID_ENABLE_PUBLISHER,1);
			cross_sub_ = nh_.subscribe(CROSS_DETECTED_SUBSCRIBER, 1, &GocellAction::callback_cross_detected, this);
			barcode_sub_ = nh_.subscribe(BAR_CODE_SUBSCRIBER, 1, &GocellAction::callback_barcode_detected, this);
            encoder_left_sub_ = nh_.subscribe(ENCODER_LEFT_SUBSCRIBER, 1, &GocellAction::callback_encoder, this);


			counter_=0;
			success_=FALSE;
			as_.start();
		}

		~GocellAction(void)
		{
		}


        void callback_encoder(const msgs::IntStampedPtr& msg)
        {
            // Store the value of the encoder
            encoder_val_=msg->data;


        }


		void callback_barcode_detected(const std_msgs::StringPtr& msg)
		{
			ROS_INFO("Barcode detected");
			barcode_value_=msg->data;
			if(goal_.cell_name==barcode_value_)
			{
				success_=TRUE;
				// Send to PID to stop publishing
				std_msgs::Bool pid_message;
				pid_message.data=FALSE;
				pid_pub_.publish(pid_message);
			}
		}


		void callback_cross_detected(const std_msgs::BoolConstPtr& msg)
		{
			ROS_INFO("cross detected");
			bool stop=FALSE;
			if(goal_.cell_name=="")
			{
				stop=TRUE;
			}
			if(goal_.cell_name==barcode_value_)
			{
				stop=TRUE;
			}
			if(stop)
			{
				success_=msg->data;
				// Send to PID to stop publishing
//				std_msgs::Bool pid_message;
//				pid_message.data=FALSE;
//				pid_pub_.publish(pid_message);
			}
		}
		void callback_image(const sensor_msgs::ImageConstPtr& msg)
		{
			// Copy the message
			// image_message_=msg;
			// ROS_INFO("Image received");
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
			image_=cv_ptr->image;
			// counter_=counter_+1;
		}
		void executeCB(const line_til_cross_action::GocellGoalConstPtr &goal)
		{
			// helper variables
			goal_.cell_name=goal->cell_name;
			success_ = false;
			// Publish the image to the line_detector
			// Publish output image
			//If the image is not empty
			counter_=0;
			barcode_value_="";
			std_msgs::Bool pid_message;
			pid_message.data=TRUE;
			pid_pub_.publish(pid_message);
			ros::Rate r(25);
			while (ros::ok())
			{
                if(counter_>1000)
				{
					break;
				}
				if(success_)
				{
					break;
				}
				if(image_.data)
				{
					// ROS_INFO("publishing image");
					cv_bridge::CvImage cvi;
					cvi.header.stamp = ros::Time::now();
					cvi.header.frame_id = "image";
					cvi.encoding = "bgr8";
					cvi.image=image_;
					sensor_msgs::Image im;
					cvi.toImageMsg(im);
					image_pub_.publish(im);
				}
				counter_++;
				r.sleep();
			}
			//result_.sequence = feedback_.sequence;
			result_.cell_final="final_robot";
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded

			if(success_){

                // Continue till the cross
                int init_encoder=encoder_val_;
                ros::Rate s(25);
                while (ros::ok())
                {
		    	if(image_.data)
			{
				// ROS_INFO("publishing image");
				cv_bridge::CvImage cvi;
				cvi.header.stamp = ros::Time::now();
				cvi.header.frame_id = "image";
				cvi.encoding = "bgr8";
				cvi.image=image_;
				sensor_msgs::Image im;
				cvi.toImageMsg(im);
				image_pub_.publish(im);
			}	


                    //Continue navigating unless advanced all the desired distance
                    if(encoder_val_>(init_encoder+ 370))
                    {
                        break;
                    }
                    if(encoder_val_<init_encoder)
                    {
                        ROS_ERROR("Overflow on the encoder");
                        std::cout<<"Last value of encoder_val= "<<init_encoder<<std::endl;
			break;
                    }

                    s.sleep();
                }

				as_.setSucceeded(result_);
				pid_message.data=FALSE;
				pid_pub_.publish(pid_message);

            }else{
				std_msgs::Bool pid_message;
				// abort and stop publishing
				pid_message.data=FALSE;
				pid_pub_.publish(pid_message);
				as_.setAborted(result_);
			}
		}
};
	int main(int argc, char** argv)
	{
		ros::init(argc, argv, "Gocell");
		GocellAction gocell(ros::this_node::getName());
		ros::spin();
		return 0;
	}

