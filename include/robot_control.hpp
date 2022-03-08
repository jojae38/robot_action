#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Char.h"
#include <time.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_action/robot_actionAction.h>
#include <thread>
using std::thread;

using namespace std;

const double camera_width = 0.44;//meter
const double camera_length = 0.34;//meter
const double Robot_wheel_r = 0.11;//meter
const double Robot_center_to_camera_center = 0.42;//meter
int MARKER_FULL_num= 30000;

#define PI 3.141592
ros::Subscriber pose_vel_sub;
ros::Publisher cmd_vel_pub;
ros::Publisher pose_vel_pub;
ros::Subscriber key_input;
enum MODE {Stop,Front,Right,Left,Back,Init};
enum MARKER_MODE {No_Marker,Find_Marker,Goto_Marker,Position_adjust};
struct POSITION
{
    double x;
    double y;
    double th;
    int order_num;
};
struct COLOR{
    uchar R;
    uchar G;
    uchar B;
    int offset;
};
struct Visual_Map{
    int cross;
    int map_row;
    int map_col;
    int row_block;
    int col_block;
};
void keycallback(const std_msgs::Char::ConstPtr &msg);
class Pioneer
{
    private:
    double speed;
    double angle_speed;
    //when the start button is pushed
    ros::Time start_time;
    //when the pause button is pushed
    ros::Time pause_time;
    //when the resume button is pushed
    ros::Time resume_time;
    // ros::Time prev_time;
    // ros::Time current_time;

    int prev_mode;
    int mode;
    int prev_Marker_mode;
    int Marker_mode;
    int MARKER_pixel;
    bool Arrive;
    struct COLOR MARKER_COLOR;
    struct COLOR ROBOT_COLOR_X;
    struct COLOR ROBOT_COLOR_Y;
    struct COLOR ORDER_COLOR;

    struct POSITION ROBOT_POS;
    struct Visual_Map Map;
    geometry_msgs::Twist vel_msg;
    nav_msgs::Odometry odom_msg;

    int marker_num;
    int order_num;
    POSITION ROBOT;
    vector<POSITION> Move_Order;
    vector<POSITION> MARKER;
    vector<POSITION> PATH;
    cv::VideoCapture capture;

    //ACTION//
    public:
    Pioneer();
    ~Pioneer();
    //Robot Movement
    void Get_param();
    void set_Position(struct POSITION &pos,double x,double y,double th);
    bool set_mode(int num);
    void set_cmd_vel(double x,double th);
    void go_front();
    void turn_left();
    void turn_right();
    void stop();
    void back();
    void run_robot();
    bool update_ROBOT_Position(const nav_msgs::Odometry::ConstPtr &msg);
    void add_path_or_marker(vector<POSITION> &Pos, int x,int y,int order_num);

    void is_direction_match();
    void is_destination_arrive();
    void keycallback(const std_msgs::Char::ConstPtr &msg);
    //Camera Part
    bool is_marker_on_sight();
    bool Publish_image();
    bool run_camera();
    
    //visualize
    void set_Visual_map(int cross,int row,int col);
    void set_color(struct COLOR &color,int R,int G,int B,int offset);
    void visualize();
    void draw_robot_at(double x,double y,double th,cv::Mat *map);
    void draw_marker_at(double x,double y,cv::Mat &map,struct COLOR color);
    int convert_world_pos_x(double x);
    int convert_world_pos_y(double y);

    void executeCB(const robot_action::robot_actionGoalConstPtr &goal)
  {
    ROS_INFO("spin");
  }
};
  
Pioneer::Pioneer()
{ 
    mode=MODE::Stop;
    prev_mode=MODE::Stop;

    cv::VideoCapture cap(0);
    if(!cap.isOpened())
		std::cerr<<"Camera open failed!"<<std::endl;
    else
        ROS_INFO("Camera model [ELP-USBFHD06H-L21] connected");
    
    capture=cap;    
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    MARKER_pixel=0;
    set_color(MARKER_COLOR,290,-10,-10,100);
    set_color(ROBOT_COLOR_X,100,200,100,0);
    set_color(ROBOT_COLOR_Y,100,100,200,0);
    set_color(ORDER_COLOR,20,20,235,0);
    set_Position(ROBOT,0,0,0);
    set_Visual_map(14,700,700);
    order_num=0;
    Arrive=false;
    //Server Part//
    
    ROS_INFO("Starting Pioneer");
}
Pioneer::~Pioneer()
{
    cv::destroyAllWindows();
    ROS_INFO("END Pioneer");
}
void Pioneer::Get_param()
{
    ros::NodeHandle nh_;
    key_input=nh_.subscribe("key_input",10,&Pioneer::keycallback,this);
    cmd_vel_pub=nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);

    ros::NodeHandle nh_private("~");
    nh_private.param<double>("speed", speed, 0.2); 
    nh_private.param<double>("angle_speed", angle_speed, 0.1);
    angle_speed*=PI;
    nh_private.param<int>("marker_num", marker_num, 10); 
    nh_private.param<int>("order_num", order_num, 5);
    
    string marker;
    string marker_pos;
    string order;
    string order_pos;
    for(int i=1;i<=marker_num;i++)
    {
        int x=0;
        int y=0;
        marker="Marker";
        marker=marker+to_string(i);
        nh_private.param<string>(marker, marker_pos,"(10,10)");
        int index=marker_pos.find(',');
        x=atoi(marker_pos.substr(index+1).c_str());
        marker_pos.erase(index,marker_pos.size()-1);
        y=atoi(marker_pos.c_str());
        add_path_or_marker(MARKER,x,y,i);
        
    }
    ROS_INFO("%d MARKER SET",marker_num);
    for(int i=1;i<=order_num;i++)
    {
        int x=0;
        int y=0;
        order="Order";
        order=order+to_string(i);
        nh_private.param<string>(order,order_pos,"(10,10)");
        int index=order_pos.find(',');
        x=atoi(order_pos.substr(index+1).c_str());
        order_pos.erase(index,order_pos.size()-1);
        y=atoi(order_pos.c_str());
        add_path_or_marker(Move_Order,x,y,i);

    }
    ROS_INFO("%d ORDER_SET",order_num);
}
void Pioneer::set_Position(struct POSITION &pos,double x,double y,double th)
{
    pos.x=x;
    pos.y=y;
    pos.th=th;
}
void Pioneer::go_front()
{
    set_cmd_vel(speed,0);
}
void Pioneer::turn_left()
{
    set_cmd_vel(0,angle_speed);
}
void Pioneer::turn_right()
{
    set_cmd_vel(0,-angle_speed);
}
void Pioneer::stop()
{
    set_cmd_vel(0,0);
}
void Pioneer::back()
{
    set_cmd_vel(-speed,0);
}
void Pioneer::add_path_or_marker(vector<POSITION> &Pos, int x,int y,int order_num)
{
    POSITION temp;
    set_Position(temp,x,y,0);
    temp.order_num=order_num;
    Pos.push_back(temp);
}
bool Pioneer::is_marker_on_sight()//30% of marker is shown
{
    if(MARKER_pixel>=MARKER_FULL_num*0.85)//85% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
        return true;   
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.8&&Marker_mode==MARKER_MODE::Goto_Marker)//hysteresis - Goto
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
        return true;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.3)//30% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Find_Marker;
        return true;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.2&&Marker_mode==MARKER_MODE::Find_Marker)//hysteresis - Find
    {
        Marker_mode=MARKER_MODE::Find_Marker;
        return true;
    }
    else
    {
        Marker_mode=MARKER_MODE::No_Marker;
        return false;
    }
}
bool Pioneer::set_mode(int num)
{
    if(mode!=num)
    {
        mode=num;
        return true;
    }
    return false;
}
void Pioneer::set_cmd_vel(double x,double th)
{
    vel_msg.linear.x=x;
    vel_msg.angular.z=th;
}
bool Pioneer::run_camera()
{
    
    cv::Mat frame;
    cv::Mat resized_frame;
    capture>>frame;
	if(frame.empty())
	    return -1;
    cv::namedWindow("frame");
    cv::moveWindow("frame",10,0);

    int marker_value=0;
    /*RGB*/
    // for(int i=1;i<frame.rows-1;i++)
    // {
    //     for(int j=1;j<frame.cols-1;j++)
    //     {
    //         int R=frame.at<cv::Vec3b>(i,j)[2];
    //         int G=frame.at<cv::Vec3b>(i,j)[1];
    //         int B=frame.at<cv::Vec3b>(i,j)[0];

    //         if(R>MARKER_COLOR.R-MARKER_COLOR.offset&&G<MARKER_COLOR.G+MARKER_COLOR.offset&&B<MARKER_COLOR.B+MARKER_COLOR.offset)
    //         {
    //             frame.at<cv::Vec3b>(i,j)[2]=255;
    //             frame.at<cv::Vec3b>(i,j)[1]=0;
    //             frame.at<cv::Vec3b>(i,j)[0]=0;
    //             marker_value++;
    //         }    
    //     }
    // }
    /*RGB*/

    cv::Mat mod_frame;
    cv::cvtColor(frame,mod_frame,cv::COLOR_BGR2HSV);
    if(Marker_mode==MARKER_MODE::Goto_Marker)
    {
        double x_pos=0;
        double y_pos=0;
        for(int i=1;i<frame.rows-1;i++)
        {
            for(int j=1;j<frame.cols-1;j++)
            {
                int H=mod_frame.at<cv::Vec3b>(i,j)[0];
                int S=mod_frame.at<cv::Vec3b>(i,j)[1];
                int V=mod_frame.at<cv::Vec3b>(i,j)[2];
                
                if(120>=H&&H>=98&&S>125&&V>145)
                {
                    frame.at<cv::Vec3b>(i,j)={0,0,255};
                    marker_value++;
                    x_pos+=double(i)/double(MARKER_pixel);
                    y_pos+=double(j)/double(MARKER_pixel);
                }    
            }
        }
        int size=125;
        for(int i=x_pos-size;i<x_pos+size;i++)
        {
            if(i<0||i>=mod_frame.rows)
            {
                continue;
            }
            else
            {
                if(y_pos+size>mod_frame.cols)
                {
                    y_pos=mod_frame.cols-size-1;
                }
                if(y_pos-size<0)
                {
                    y_pos=size;
                }
                frame.at<cv::Vec3b>(i,y_pos+size)={0,255,0};
                frame.at<cv::Vec3b>(i,y_pos-size)={0,255,0};
                
            }
        }
        for(int i=y_pos-size;i<y_pos+size;i++)
        {
            if(i<0||i>=mod_frame.cols)
            {
                continue;
            }
            else
            {
                if(x_pos+size>mod_frame.rows)
                {
                    x_pos=mod_frame.rows-size-1;
                }
                if(x_pos-size<0)
                {
                    x_pos=size;
                }
                frame.at<cv::Vec3b>(x_pos+size,i)={0,255,0};
                frame.at<cv::Vec3b>(x_pos-size,i)={0,255,0};
            }
        }
        ROS_INFO("X_POS = %d",int(x_pos));
        ROS_INFO("Y_POS = %d",int(y_pos));

        MARKER_pixel=marker_value;
    }/*HSV*/
    else
    {
    for(int i=1;i<frame.rows-1;i++)
    {
        for(int j=1;j<frame.cols-1;j++)
        {
            int H=mod_frame.at<cv::Vec3b>(i,j)[0];
            int S=mod_frame.at<cv::Vec3b>(i,j)[1];
            int V=mod_frame.at<cv::Vec3b>(i,j)[2];

            if(120>=H&&H>=98&&S>125&&V>145)
            {
                frame.at<cv::Vec3b>(i,j)={0,0,255};
                // cout <<S<<endl;
                marker_value++;
            }    
        }
    }
    MARKER_pixel=marker_value;
    // ROS_INFO("Marker_value %d",MARKER_pixel);
    // number++;
    }   
    cv::imshow("frame",frame);
    /*HSV*/

    /*RGB RESIZED - if frame is to big to run*/
    // double resize_rate=0.1;
    // cv::resize(frame,resized_frame,cv::Size(),resize_rate,resize_rate);
    // for(int i=1;i<resized_frame.rows-1;i++)
    // {
    //     for(int j=1;j<resized_frame.cols-1;j++)
    //     {
    //         int R=resized_frame.at<cv::Vec3b>(i,j)[2];
    //         int G=resized_frame.at<cv::Vec3b>(i,j)[1];
    //         int B=resized_frame.at<cv::Vec3b>(i,j)[0];

    //         if(R>MARKER_COLOR.R-MARKER_COLOR.offset&&G<MARKER_COLOR.G+MARKER_COLOR.offset&&B<MARKER_COLOR.B+MARKER_COLOR.offset)
    //         {
    //             resized_frame.at<cv::Vec3b>(i,j)[2]=255;
    //             resized_frame.at<cv::Vec3b>(i,j)[1]=0;
    //             resized_frame.at<cv::Vec3b>(i,j)[0]=0;
    //             marker_value++;
    //         }    
    //     }
    // }
    // cv::imshow("frame",resized_frame);
    /*RGB RESIZED*/
    // cout << marker_value<<endl;
    Pioneer::visualize();
    if(cv::waitKey(10)==27)
        return 0;
    return true;
}
void Pioneer::set_color(struct COLOR &color,int R,int G,int B,int offset)
{
    color.R=R;
    color.G=G;
    color.B=B;
    color.offset=offset;
}
void Pioneer::set_Visual_map(int cross,int row,int col)
{
    Map.cross=cross;
    Map.map_row=row;
    Map.map_col=col;
    Map.row_block=Map.map_row/cross;
    Map.col_block=Map.map_col/cross;
}
void Pioneer::visualize()
{
    
    //Make blank map
    
    cv::Mat map(cv::Size(Map.map_row,Map.map_col),CV_8UC3,{0,0,0});
    //Make grid map
    for(int i=1;i<Map.cross;i++)
    {
        for(int k=0;k<Map.map_col;k++)
        {
            map.at<cv::Vec3b>(i*Map.row_block,k)={255,255,255};
        }
        for(int k=0;k<Map.map_row;k++)
        {
            map.at<cv::Vec3b>(k,i*Map.col_block)={255,255,255};
        }
    }
    // Print Marker(whick is found)
    for(int i=0;i<Pioneer::MARKER.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_y(Pioneer::MARKER[i].y),convert_world_pos_x(Pioneer::MARKER[i].x),map,MARKER_COLOR);
    }
    // //Print Order Seq
    for(int i=0;i<Pioneer::Move_Order.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_y(Pioneer::Move_Order[i].y),convert_world_pos_x(Pioneer::Move_Order[i].x),map,ORDER_COLOR);
    }
    // //Print Robot
    Pioneer::draw_robot_at(convert_world_pos_y(ROBOT.y),convert_world_pos_x(ROBOT.x),ROBOT.th,&map);
    cv::namedWindow("map");
    cv::moveWindow("map",865,0);
    cv::imshow("map",map);
}
void Pioneer::draw_robot_at(double x,double y,double th,cv::Mat *map)
{
    double cos_th=cos(ROBOT.th);
    double sin_th=sin(ROBOT.th);
    
    for(int i=-40;i<-1;i++)
    {
        for(int j=-3;j<=3;j++)
        {
            map->at<cv::Vec3b>(int(i*cos_th+j*sin_th+x),int(i*sin_th-j*cos_th+y))={ROBOT_COLOR_X.B,ROBOT_COLOR_X.G,ROBOT_COLOR_X.R};
        }
    }
    for(int i=1;i<40;i++)
    {
        for(int j=-3;j<=3;j++)
        {
            map->at<cv::Vec3b>(int(-i*sin_th-j*cos_th+x),int(i*cos_th-j*sin_th+y))={ROBOT_COLOR_Y.B,ROBOT_COLOR_Y.G,ROBOT_COLOR_Y.R};    
        }
    }
}
void Pioneer::draw_marker_at(double x,double y,cv::Mat &map,struct COLOR color)
{
    for(int i=-5;i<=5;i++)
    {
        for(int j=-5;j<=5;j++)
        {
            map.at<cv::Vec3b>(i+x,j+y)={color.B,color.G,color.R};
        }
    }
}
int Pioneer::convert_world_pos_x(double x)
{
    double world_x=Map.row_block*(x+1);
    return int(world_x);
}
int Pioneer::convert_world_pos_y(double y)
{
    double world_y=Map.col_block*(double(Map.cross-1)-y);
    return int(world_y);
}
void Pioneer::is_direction_match()
{
    int temp_x=Move_Order[order_num].x-ROBOT.x;
    int temp_y=Move_Order[order_num].y-ROBOT.y;
    double temp_th=acos(temp_x/sqrt((temp_x*temp_x+temp_y*temp_y)));
    if(ROBOT.th<temp_th+0.1||ROBOT.th>temp_th-0.1)
    {
        ROS_INFO("Direction Matched");
        mode=MODE::Front;
    }
    else
    {
        if(ROBOT.th<temp_th)
            mode=MODE::Left;
        else
            mode=MODE::Right;
    }
}
void Pioneer::is_destination_arrive()
{
    if(ROBOT.x<Move_Order[order_num].x+0.1||ROBOT.x>Move_Order[order_num].x-0.1)
    {
        if(ROBOT.y<Move_Order[order_num].y+0.1||ROBOT.y>Move_Order[order_num].y-0.1)
        {
            order_num++;
            if(Move_Order[order_num].order_num==999)
            {
                Arrive=true;
            }
        }
    }
}
void Pioneer::run_robot()
{   
    // ros::Rate rate(20);
    // while(ros::ok())
    // {
        // run_camera();
        // is_marker_on_sight();
        if(mode!=prev_mode)
        {
            ROS_INFO("MODE_CHANGE TO %d",mode);
            prev_mode=mode;
        }
        // Marker
        if(Marker_mode==MARKER_MODE::Position_adjust)
        {
            
        }
        else if(Marker_mode==MARKER_MODE::Goto_Marker)
        {
            // Pioneer::stop();
            ROS_INFO("SEE MOST MARKER");
            //calculate middle pos
            //set to marker location
        }
        else if(Marker_mode==MARKER_MODE::Find_Marker)
        {
            // Pioneer::stop();
            ROS_INFO("SEE MARKER");
            //calculate middle pos
            //set to marker location
        }
        else if(Marker_mode==MARKER_MODE::No_Marker)
        {
            // Pioneer::stop();
            // ROS_INFO("STOP");
        }
        // is_direction_match();
        // is_destination_arrive();
        // RUN
        if(mode==MODE::Stop)
        {
            Pioneer::stop();
        }
        else if(mode==MODE::Front)
        {
            Pioneer::go_front();

        }
        else if(mode==MODE::Left)
        {
            Pioneer::turn_left();

        }
        else if(mode==MODE::Right)
        {
            Pioneer::turn_right();

        }
        else if(mode==MODE::Back)
        {
            Pioneer::back();
        }
        cmd_vel_pub.publish(vel_msg);
        // mode=MODE::Stop;
    //     rate.sleep();
    //     ros::spinOnce();
    // }
}
bool Pioneer::update_ROBOT_Position(const nav_msgs::Odometry::ConstPtr &msg)
{
    //MATCH THIS DATA TO ROSARIA/POSE DATA
    //x and y is ok but oriantation w? z? th is unstable rotate 90 degree and compare
    odom_msg=*msg;
    ROBOT.x=msg->pose.pose.position.x;
    ROBOT.y=msg->pose.pose.position.y;


    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose,pose);
    //Turn Quaternion to Euler
    ROBOT.th=tf::getYaw(pose.getRotation());
    return true;
}

void Pioneer::keycallback(const std_msgs::Char::ConstPtr &msg)
{
    if(msg->data=='w'||msg->data=='w')
    {
        mode=MODE::Front;
        ROS_INFO("CHANGE_MODE -> FRONT");
    }
    else if(msg->data=='s'||msg->data=='S')
    {
        mode=MODE::Stop;
        ROS_INFO("CHANGE_MODE -> STOP");
    }
    else if(msg->data=='d'||msg->data=='D')
    {
        mode=MODE::Right;
        ROS_INFO("CHANGE_MODE -> RIGHT");
    }
    else if(msg->data=='a'||msg->data=='A')
    {
        mode=MODE::Left;
        ROS_INFO("CHANGE_MODE -> LEFT");
    }
    else if(msg->data=='x'||msg->data=='X')
    {
        mode=MODE::Back;
        ROS_INFO("CHANGE_MODE -> BACK");
    }
    ROS_INFO("GET KEY %c",msg->data);
}

class Robot_Action
{
    protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot_action::robot_actionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    robot_action::robot_actionActionFeedback feedback_;
    robot_action::robot_actionActionResult result_;

    public:

    Robot_Action(std::string name):as_(nh_,name,boost::bind(&Robot_Action::executeCB,this,_1),false),action_name_(name){as_.start();}
    ~Robot_Action(void)
    {}

    void executeCB(const robot_action::robot_actionActionConstPtr &goal)
    {
        ROS_INFO("RUN");
    }
    void run()
    {

    }
    
};