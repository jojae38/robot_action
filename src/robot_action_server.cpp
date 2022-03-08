#include "robot_control.hpp"

int main(int argc, char** argv)
{
  
  Pioneer Pioneer_(argc,argv);
  Pioneer_.run_robot();
  // RobotAction_server robot_action("robot_action_server");

  return 0;
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