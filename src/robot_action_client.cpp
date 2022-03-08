#include "robot_control.hpp"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_action_client");
  int order_num=0;
  actionlib::SimpleActionClient<robot_action::robot_actionAction> robot_client("robot_client",true);
  ROS_INFO("Waiting for Robot_action_server to start.");
  robot_client.waitForServer();
  ROS_INFO("Robot_action_server_ON, sending goal.");
  robot_action::robot_actionGoal goal;
  
  //Action init
  bool finished_before_timeout = robot_client.waitForResult(ros::Duration(1.0));
  // std::cout <<"Type in Order number: ";
  // while(true)
  // {
  //   std::cin>>order_num;
  //   if(order_num>=1&&order_num<=10)
  //   {
  //     ROS_INFO("Order_number: %d",order_num);
  //     break;
  //   }
  //   else
  //   {
  //     ROS_INFO("Over Value or Not int please try again");
  //   }
  // }
  while(order_num)
  {

  }
  while(!finished_before_timeout)
  {
      finished_before_timeout = robot_client.waitForResult(ros::Duration(1.0));
      robot_client.sendGoal(goal);
      ROS_INFO("Not done Yet");
  }
  
  actionlib::SimpleClientGoalState state = robot_client.getState();
  ROS_INFO("FINISHED: %s",state.toString().c_str());
  


  return 0;
}