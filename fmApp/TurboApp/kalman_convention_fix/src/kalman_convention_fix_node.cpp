#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class Kalman_convention_fix
{
    typedef std::string framename;
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle n_;

    ros::Duration pubtimer_duration_;
    ros::Timer pubtimer_;
    tf::TransformBroadcaster tfbroadcast_;
    tf::TransformListener tflistener_;

    std::string frame_lps_;
    std::string frame_link_kalman_; // This is the frame that kalman publishes
    std::string frame_odom_;
    std::string frame_link_odom_;


    void publish_tf(void)
    {
        tf::Transform transform;
        tf::StampedTransform tf_odom, tf_kalman;
        try{
          // tflistener_.waitForTransform(frame_link_kalman_, frame_lps_, ros::Time(0), ros::Duration(1.0) );
          // listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
          tflistener_.lookupTransform(frame_link_kalman_, frame_lps_, ros::Time(0), tf_odom);
          tflistener_.lookupTransform(frame_link_odom_, frame_odom_, ros::Time(0), tf_kalman);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s: %s", ros::this_node::getName() , ex.what());
        }
        transform = tf_odom.inverseTimes(tf_kalman);
        tfbroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_lps_, frame_odom_));
    }
    void setupNode()
    {
        // Timer
        double pubtimer_duration_sec;
        nh_.param<double>("pubtimer_duration_sec", pubtimer_duration_sec, 0.1);
        pubtimer_duration_.fromSec(pubtimer_duration_sec);
        pubtimer_ = nh_.createTimer(pubtimer_duration_, &Kalman_convention_fix::pubtimer_callback, this);

        // Frames
        nh_.param<std::string>("frame_lps", this->frame_lps_, "/lps");
        nh_.param<std::string>("frame_link_kalman", this->frame_link_kalman_, "/base_kalman");
        nh_.param<std::string>("frame_odom", this->frame_odom_, "/odom");
        nh_.param<std::string>("frame_link_odom", this->frame_link_odom_, "/base_link");
    }

    void pubtimer_callback(const ros::TimerEvent& event)
    {
      publish_tf();
    }

public:
    Kalman_convention_fix() : nh_("~")
    {
        setupNode();
    }

    void spin()
    {
      ros::spin();
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Kalman_convention_fix");
  Kalman_convention_fix n;
  n.spin();
}
