#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <lift_tipper/tipperAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "lift_tipper");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<lift_tipper::tipperAction> ac("tipper", true);

  std::cout<<"Client started"<<std::endl;
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  //learning_actionlib::FibonacciGoal goal;
  lift_tipper::tipperGoal goal;

  goal.lift=true;

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}



