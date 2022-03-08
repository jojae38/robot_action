#include "robot_control.hpp"
void func1();
void func2();
void func3();

Pioneer Pioneer_;
int main(int argc, char** argv)
{
  ros::init(argc,argv,"Robot_control");
  Pioneer_.Get_param();//add Marker & Order & speed & angle_speed
  // RobotAction_server robot_action("robot_action_server");
  thread t1(func1);
  thread t2(func2);
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  t1.join();
  t2.join();
  return 0;
}
void func1()
{
  ros::Rate rate_1(10);
  while(ros::ok())
  {
    Pioneer_.run_camera();
    rate_1.sleep();
    ROS_INFO("func1 activated");
  }
}
void func2()
{
  ros::Rate rate_2(20);
  while(ros::ok())
  {
    Pioneer_.run_robot();
    rate_2.sleep();
    ROS_INFO("func2 activated");
  }
}
void func3()
{
  ros::Rate rate_3(100);
  while(ros::ok())
  {
    Robot_Action Robot_Action_("Robot_action");
    rate_3.sleep();
    ROS_INFO("func2 activated");
  }
}
// Pioneer Pioneer_;


// int main(int argc, char **argv)
// {
//     ros::init(argc,argv,"Robot_control");
//     ros::NodeHandle n;

//     pose_vel_sub=n.subscribe("/RosAria/pose",10,poseCallback);
//     key_input=n.subscribe("key_input",10,keycallback);
//     cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
//     // pose_vel_pub=n.advertise<nav_msgs::Odometry>("/RosAria/pose",1);
//     Pioneer_.Get_param();
//     cv::VideoCapture cap(0);
//     if(!cap.isOpened())
// 		std::cerr<<"Camera open failed!"<<std::endl;
//     else
//         ROS_INFO("Camera model [ELP-USBFHD06H-L21] connected");
        
//     Pioneer_.run_robot(cap);
//     return 0;    
// }