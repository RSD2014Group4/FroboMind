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

#define IMAGE_SUBSCRIBER "usb_cam/image_raw"
#define IMAGE_PUBLISHER "line_action/image_raw"
#define PID_ENABLE_PUBLISHER "line_action/pid_enable"

#define CROSS_DETECTED_SUBSCRIBER "line_node/cross_detected"


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

  ros::Publisher image_pub_;
  ros::Publisher pid_pub_;

  ros::Subscriber image_sub_;
  ros::Subscriber cross_sub_;


  cv::Mat image_;



   bool success_;
  int counter_;


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


    counter_=0;
    success_=FALSE;

    as_.start();
  }

  ~GocellAction(void)
  {
  }




    void callback_cross_detected(const std_msgs::BoolConstPtr& msg)
    {
        ROS_INFO("cross detected");

        success_=msg->data;


        // Send to PID to stop publishing

        std_msgs::Bool pid_message;
        pid_message.data=FALSE;

        pid_pub_.publish(pid_message);


    }






    void callback_image(const sensor_msgs::ImageConstPtr& msg)
    {

            // Copy the message
//            image_message_=msg;

//            ROS_INFO("Image received");

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

    success_ = false;


    // Publish the image to the line_detector
    // Publish output image

    //If the image is not empty

    counter_=0;

    std_msgs::Bool pid_message;
    pid_message.data=TRUE;

    pid_pub_.publish(pid_message);


    ros::Rate r(25);
    while (ros::ok())
    {
        if(counter_>100)
        {
            break;
        }

        if(success_)
        {
            break;
        }

        if(image_.data)
        {
                ROS_INFO("publishing image");
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
            as_.setSucceeded(result_);
         }else{
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













////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Old code///////////////////////////////////////////////////////////////////////////////////////

//Execute with feedbacks

//void executeCB(const line_til_cross_action::GocellGoalConstPtr &goal)
//{
//  // helper variables
//  ros::Rate r(10);
//  bool success = true;

//  // publish info to the console for the user
//  //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

////  ROS_INFO("in the executeCB");


//  // start executing the action
//  for(int i=0; i<=2; i++)
//  {
//    // check that preempt has not been requested by the client
//    if (as_.isPreemptRequested() || !ros::ok())
//    {
//      ROS_INFO("%s: Preempted", action_name_.c_str());
//      // set the action state to preempted
//      as_.setPreempted();
//      success = false;
//      break;
//    }
//    //feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);

//    feedback_.cell_current="stil working";


//    ROS_INFO("Get inside here");

//    // publish the feedback
//    as_.publishFeedback(feedback_);
//    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
//    r.sleep();
//  }

//  if(success)
//  {

//    //result_.sequence = feedback_.sequence;
//      result_.cell_final="final_robot";
//    ROS_INFO("%s: Succeeded", action_name_.c_str());
//    // set the action state to succeeded
//    as_.setSucceeded(result_);
//  }
//}


//};






























//#include <ros/ros.h>
//#include <std_msgs/Float32.h>
//#include <actionlib/server/simple_action_server.h>
//#include <line_til_cross_action/GocellAction.h>
////#include <learning_actionlib/AveragingAction.h>




//class GocellAction
//{
//public:

//  GocellAction(std::string name) :
//    as_(nh_, name, false),
//    action_name_(name)
//  {
//    //register the goal and feeback callbacks

////   as_= server(n, "go_til_cell", boost::bind(&execute, _1, &server), false);

//    as_.registerGoalCallback(boost::bind(&GocellAction::goalCB, this));
//    as_.registerPreemptCallback(boost::bind(&GocellAction::preemptCB, this));
//    //subscribe to the data topic of interest
//    //sub_ = nh_.subscribe("/random_number", 1, &GocellAction::analysisCB, this);

//    as_.start();

//  }

//  ~GocellAction(void)
//  {
//  }

//  void goalCB()
//  {

//    ROS_INFO("In goalCB");
//    // reset helper variables
//    data_count_ = 0;
//    sum_ = 0;
//    sum_sq_ = 0;
//    // accept the new goal
//    goal_ = as_.acceptNewGoal()->dishwasher_id;

//      ROS_INFO("In goal cb");

//      // set the action state to succeeded
//   //   result_.total_dishes_cleaned=5;
//      result_.total_dishes_cleaned=5;

//      as_.setSucceeded();
//     goal_ = as_.acceptNewGoal()->dishwasher_id;


//  }


//  void preemptCB()
//  {
//    ROS_INFO("%s: Preempted", action_name_.c_str());
//    // set the action state to preempted
//    as_.setPreempted();
//  }
///*
//  void analysisCB(const std_msgs::Float32::ConstPtr& msg)
//  {
//    // make sure that the action hasn't been canceled
//    if (!as_.isActive())
//      return;

//    ROS_INFO("%s: Succeeded", action_name_.c_str());
//    // set the action state to succeeded
//    result_.total_dishes_cleaned=5;

//    as_.setSucceeded();


//  }
//*/
//protected:

//  ros::NodeHandle nh_;
//  actionlib::SimpleActionServer<line_til_cross_action::GocellAction> as_;
//  std::string action_name_;

//  int data_count_, goal_;
//  float sum_, sum_sq_;
//  line_til_cross_action::GocellFeedback feedback_;
//  line_til_cross_action::GocellResult result_;
//  ros::Subscriber sub_;
//};

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "go_til_cell");
//  ROS_INFO("Started server");
//  GocellAction Gocell(ros::this_node::getName());
//   ROS_INFO("Gocell_created");
//  ros::spin();

//  return 0;
//}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Old code///////////////////////////////////////////////////////////////////////////////////////

/*
#include <line_til_cross_action/GocellAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;

typedef actionlib::SimpleActionServer<line_til_cross_action::GocellAction> Server;

void execute(const line_til_cross_action::GocellGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here


    // Subscribe to line_crosses

    as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_til_cell_server");
  ros::NodeHandle n;
  Server server(n, "go_til_cell", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
*/
