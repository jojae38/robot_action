#include "robot_control.hpp"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_action_client");
  int order_num=0;
  actionlib::SimpleActionClient<robot_action::robot_actionAction> robot_client("/ojtAction",true);
  ROS_INFO("Waiting for Robot_action_server to start.");
  robot_client.waitForServer();
  ROS_INFO("Robot_action_server_ON, sending goal.");
  robot_action::robot_actionGoal goal;
  goal.order="stop";
  robot_client.sendGoal(goal);
  ROS_INFO("first");
  ros::Duration five_second(5.0);
  five_second.sleep();
  goal.order="start";
  robot_client.sendGoal(goal);
  ROS_INFO("second");
  //Action init
  bool finished_before_timeout = robot_client.waitForResult(ros::Duration(10000.0));
  if(finished_before_timeout)
  {
    ROS_INFO("1");
  }
  else
  {
    ROS_INFO("2");
  }
  // while(!finished_before_timeout)
  // {
  //   if(goal.order=="start")
  //   {
  //     goal.order="stop";
  //   }
  //   else
  //   {
  //     goal.order="start";
  //   }
  //     finished_before_timeout = robot_client.waitForResult(ros::Duration(1.0));
  //     robot_client.sendGoal(goal);
  //     ROS_INFO("Not done Yet");
  // }
  
  actionlib::SimpleClientGoalState state = robot_client.getState();
  ROS_INFO("FINISHED: %s",state.toString().c_str());
  


  return 0;
}