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
#include <std_msgs/String.h>

using namespace std;

  cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 4.7717248258065513e+02, 0., 3.4345166112868361e+02, 0.,
                       4.7877303179533322e+02, 2.1840674677502400e+02, 0., 0., 1.);
  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -5.0679364100294742e-02, 1.9604492374058202e-01,
                        -8.2735393054366954e-04, -6.8036803338709421e-05,
                        -2.0917602681902503e-01); // Assuming no lens distortion


const double camera_width = 0.45;//meter
const double camera_length = 0.34;//meter
const double Robot_wheel_r = 0.11;//meter
const double Robot_center_to_camera_center = 0.42;//meter
int MARKER_FULL_num= 30000;

#define PI 3.141592
//ROBOT
ros::Publisher cmd_vel_pub;

ros::Subscriber key_input;

//SERVER
ros::Subscriber stop_status;
ros::Subscriber odom_status;
ros::Subscriber start_status;
ros::Publisher robot_result;
ros::Publisher robot_feedback;
enum MODE {Stop,Front,Right,Left,Back,Init};
enum MARKER_MODE {No_Marker,Find_Marker,Goto_Marker,Complete_Marker,Position_adjust};
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
    //adjust
    ros::Time adjust_start_time;
    ros::Time adjust_end_time;
    //camera pos
    int cam_pixel_x_pos;
    int cam_pixel_y_pos;
    double cam_real_x_pos;
    double cam_real_y_pos;
    double cam_real_dis;
    double cam_real_th; 
    double robot_adjust_th;
    //
    bool mode_change;
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
    bool pause_stat;
    bool start_stat;
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

    void calc_adjust_val();//When Marker is 15% shown do adjust_th first then adjust_x
    void exact_adjust_val();//When Marker is 90% shown do adjust_x first then adjust_th
    void adjust_th(double th);
    void adjust_x(double x);
    void is_direction_match();
    void is_destination_arrive();

    void keycallback(const std_msgs::Char::ConstPtr &msg);
    void pausecallback(const std_msgs::String::ConstPtr &msg);
    void startcallback(const std_msgs::String::ConstPtr &msg);
    void odomcallback(const nav_msgs::Odometry::ConstPtr &msg);
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
    // int convert_robot_pos_x(double x);
    // int convert_robot_pos_y(double y);
};
  
Pioneer::Pioneer()
{ 
    mode=MODE::Stop;
    pause_stat=false;
    start_stat=false;
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
    //Robot
    key_input=nh_.subscribe("key_input",10,&Pioneer::keycallback,this);
    stop_status=nh_.subscribe("/pause",1,&Pioneer::pausecallback,this);
    start_status=nh_.subscribe("/Start",1,&Pioneer::startcallback,this);
    odom_status=nh_.subscribe("/RosAria/pose",1,&Pioneer::odomcallback,this);
    cmd_vel_pub=nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
    // cmd_vel_cal=nh_.advertise<geometry_msgs::Twist>("",10);
    //Server
    robot_result=nh_.advertise<std_msgs::String>("",1);
    robot_feedback=nh_.advertise<std_msgs::String>("",10);

    ros::NodeHandle nh_private("~");
    nh_private.param<double>("speed", speed, 0.2); 
    nh_private.param<double>("angle_speed", angle_speed, 0.1);
    // angle_speed*=PI;
    nh_private.param<int>("marker_num", marker_num, 10); 
    nh_private.param<int>("order_num", order_num, 5);
    
    string marker;
    string marker_pos;
    string order;
    string order_pos;

    string robot_pos;
    nh_private.param<string>("ROBOT_POS", robot_pos,"0,0");
    int x=0;
    int y=0;
    int index=0;
    index=robot_pos.find(',');
    x=atoi(robot_pos.substr(index+1).c_str());
    robot_pos.erase(index,robot_pos.size()-1);
    y=atoi(robot_pos.c_str());
    cout<<y<<endl;
    ROBOT.x=x;
    ROBOT.y=y;
    ROS_INFO("STARTING ROBOT_x: %f ROBOT_y: %f ROBOT_th: %f",ROBOT.y,ROBOT.x,ROBOT.th);
    for(int i=1;i<=marker_num;i++)
    {
        x=0;
        y=0;
        marker="Marker";
        marker=marker+to_string(i);
        nh_private.param<string>(marker, marker_pos,"10,10");
        index=marker_pos.find(',');
        x=atoi(marker_pos.substr(index+1).c_str());
        marker_pos.erase(index,marker_pos.size()-1);
        y=atoi(marker_pos.c_str());
        add_path_or_marker(MARKER,x,y,i);
        
    }
    ROS_INFO("%d MARKER SET",marker_num);
    for(int i=1;i<=order_num;i++)
    {
        x=0;
        y=0;
        order="Order";
        order=order+to_string(i);
        nh_private.param<string>(order,order_pos,"10,10");
        index=order_pos.find(',');
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
bool Pioneer::is_marker_on_sight()//90% of marker is shown
{
    if(MARKER_pixel>=MARKER_FULL_num*0.9)//90% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
        return true;   
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.85&&Marker_mode==MARKER_MODE::Goto_Marker)//hysteresis - Goto
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
        return true;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.15)//15% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Find_Marker;
        return true;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.1&&Marker_mode==MARKER_MODE::Find_Marker)//hysteresis - Find
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
     Pioneer::visualize();
    // cv::Mat un_proceed_frame;
    cv::Mat frame;
    // cv::Mat resized_frame;
    capture>>frame;
	if(frame.empty())
	    return -1;
    // cv::namedWindow("un_proceed_frame");
    // cv::moveWindow("un_proceed_frame",865,0);

    cv::namedWindow("frame");
    cv::moveWindow("frame",10,0);
    // undistort(un_proceed_frame,frame,camMatrix,distCoeffs);
    
    int marker_value=0;
    
    cv::Mat mod_frame;
    cv::cvtColor(frame,mod_frame,cv::COLOR_BGR2HSV);
    if(Marker_mode==MARKER_MODE::Find_Marker||Marker_mode==MARKER_MODE::Goto_Marker)
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

                if(120>=H&&H>=98&&S>105&&V>125)
                {
                    frame.at<cv::Vec3b>(i,j)={0,0,255};
                    marker_value++;
                    y_pos+=double(i)/double(MARKER_pixel);
                    x_pos+=double(j)/double(MARKER_pixel);
                }    
            }
        }
        
        Arrive=true;
        // ROS_INFO("X_POS = %d",int(x_pos));
        // ROS_INFO("Y_POS = %d",int(y_pos));
        cam_pixel_x_pos=x_pos;
        cam_pixel_y_pos=y_pos;
        cam_real_y_pos=400-cam_pixel_x_pos;
        cam_real_x_pos=300-cam_pixel_y_pos;

        cam_real_x_pos/=1754.4;
        cam_real_y_pos/=1754.4;
        cam_real_dis=sqrt(cam_real_x_pos*cam_real_x_pos+cam_real_y_pos*cam_real_y_pos);
        cam_real_x_pos+=Robot_center_to_camera_center;
        cam_real_th=tan(cam_real_y_pos/cam_real_x_pos);
        cout <<"AD_x: "<<cam_real_x_pos<<endl;
        cout <<"AD_y: "<<cam_real_y_pos<<endl;
        cout <<"AD_TH: "<<cam_real_th<<endl;
        
        int size=150;
        POSITION Max_x,Max_y,Min_x,Min_y;
        Max_x.x=cam_pixel_x_pos-size;
        Max_y.y=cam_pixel_y_pos-size;
        Min_x.x=cam_pixel_x_pos+size;
        Min_y.y=cam_pixel_y_pos+size;

        int x_start=cam_pixel_x_pos-size;
        int x_end=cam_pixel_x_pos+size;
        int y_start=cam_pixel_y_pos-size;
        int y_end=cam_pixel_y_pos+size;

        if(Marker_mode==MARKER_MODE::Goto_Marker&&x_end<frame.cols-1&&x_start>0&&y_end<frame.rows-1&&y_start>0)
        {   
            for(int i=y_start;i<y_end;i++)
            {
                for(int j=x_start;j<x_end;j++)
                {

                    // frame.at<cv::Vec3b>(i,j)={255,0,0};
                    int H=mod_frame.at<cv::Vec3b>(i,j)[0];
                    int S=mod_frame.at<cv::Vec3b>(i,j)[1];
                    int V=mod_frame.at<cv::Vec3b>(i,j)[2];
                    if(120>=H&&H>=98&&S>105&&V>125)
                    {
                        if(j>Max_x.x)
                        set_Position(Max_x,j,i,0);
                        if(j<Min_x.x)
                        set_Position(Min_x,j,i,0);
                        if(i>Max_y.y)
                        set_Position(Max_y,j,i,0);
                        if(i<Min_y.y)
                        set_Position(Min_y,j,i,0);
                    }    
                }
            }
            double dis1=sqrt(pow((cam_pixel_x_pos-Max_x.x),2)+pow((cam_pixel_y_pos-Max_x.y),2));
            double dis2=sqrt(pow((cam_pixel_x_pos-Min_x.x),2)+pow((cam_pixel_y_pos-Min_x.y),2));
            double dis3=sqrt(pow((cam_pixel_x_pos-Max_y.x),2)+pow((cam_pixel_y_pos-Max_y.y),2));
            double dis4=sqrt(pow((cam_pixel_x_pos-Min_y.x),2)+pow((cam_pixel_y_pos-Min_y.y),2));
            if(dis1<125&&dis2<125&&dis3<125&&dis4<125)
            {
                Marker_mode=MARKER_MODE::Complete_Marker;
                if(dis1>115&&dis2>115&&dis3>115&&dis4>115)
                {
                    double left_th=atan((Max_x.x-Max_y.x)/(Max_y.y-Max_x.y));
                    double right_th=atan((Min_x.x-Max_y.x)/(Min_y.y-Max_x.y));
                    // cout <<"Left_th: "<<left_th<<endl;
                    // cout <<"Right_th: "<<right_th<<endl;
                    if(left_th>right_th)
                    {
                        robot_adjust_th=right_th;
                    }
                    else
                    {
                        robot_adjust_th=left_th;
                    }
                    //계산
                }
                else
                {
                    robot_adjust_th=0;
                }
                cout <<"adjust_th: "<<robot_adjust_th<<endl;
            }
            
            // cout <<"dis1: "<<dis1<<endl;
            // cout <<"dis2: "<<dis2<<endl;
            // cout <<"dis3: "<<dis3<<endl;
            // cout <<"dis4: "<<dis4<<endl;
           for(int i=-3;i<=3;i++)
            {
                for(int j=-3;j<=3;j++)
                {
                    frame.at<cv::Vec3b>(Max_x.y+j,Max_x.x+i)={255,0,0};
                    frame.at<cv::Vec3b>(Min_x.y+j,Min_x.x+i)={255,0,0};
                    frame.at<cv::Vec3b>(Max_y.y+j,Max_y.x+i)={255,0,0};
                    frame.at<cv::Vec3b>(Min_y.y+j,Min_y.x+i)={255,0,0};
                }
            } 
        }
        
        // cam_real_y_pos=400-cam_pixel_x_pos;
        // cam_real_x_pos=300-cam_pixel_y_pos;

        // cam_real_x_pos/=1754.4;
        // cam_real_y_pos/=1754.4;
        // cout <<"x: "<< cam_pixel_x_pos<<endl;
        // cout <<"y: "<< cam_pixel_y_pos<<endl;
        
        
        // cout <<"ROBOT_ADJUST_TH"<<robot_adjust_th<<endl;
        // for(int i=x_pos-size;i<x_pos+size;i++)
        // {
        //     if(i<0||i>=mod_frame.rows)
        //     {
        //         continue;
        //     }
        //     else
        //     {
        //         if(y_pos+size>mod_frame.cols)
        //         {
        //             y_pos=mod_frame.cols-size-1;
        //         }
        //         if(y_pos-size<0)
        //         {
        //             y_pos=size;
        //         }
        //         frame.at<cv::Vec3b>(i,y_pos+size)={0,255,0};
        //         frame.at<cv::Vec3b>(i,y_pos-size)={0,255,0};
                
        //     }
        // }
        // for(int i=y_pos-size;i<y_pos+size;i++)
        // {
        //     if(i<0||i>=mod_frame.cols)
        //     {
        //         continue;
        //     }
        //     else
        //     {
        //         if(x_pos+size>mod_frame.rows)
        //         {
        //             x_pos=mod_frame.rows-size-1;
        //         }
        //         if(x_pos-size<0)
        //         {
        //             x_pos=size;
        //         }
        //         frame.at<cv::Vec3b>(x_pos+size,i)={0,255,0};
        //         frame.at<cv::Vec3b>(x_pos-size,i)={0,255,0};
        //     }
        // }
    

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
                // cout <<"H: "<< H<<endl;
                // cout <<"S: "<< S<<endl;
                // cout <<"V: "<< V<<endl;
                    //            if(H<10)
                    // {
                    //     frame.at<cv::Vec3b>(i,j)={0,0,255};
                    // }    
                if(120>=H&&H>=98&&S>105&&V>125)
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
    //For Calculate
    // for(int i=0;i<frame.rows;i++)
    // {
    //     for(int j=1;j<8;j++)
    //     {
    //         frame.at<cv::Vec3b>(i,j*100)={0,0,0};
    //         un_proceed_frame.at<cv::Vec3b>(i,j*100)={0,0,0};
    //     }
    // }
    // for(int i=0;i<frame.cols;i++)
    // {
    //     for(int j=1;j<6;j++)
    //     {
    //         frame.at<cv::Vec3b>(j*100,i)={0,0,0};
    //         un_proceed_frame.at<cv::Vec3b>(j*100,i)={0,0,0};

    //     }
   
    // }
       
    cv::imshow("frame",frame);
    // cv::imshow("un_proceed_frame",un_proceed_frame);
    /*HSV*/
    // cout << marker_value<<endl;
   
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
        Pioneer::draw_marker_at(convert_world_pos_y(Pioneer::MARKER[i].y),convert_world_pos_x(-Pioneer::MARKER[i].x),map,MARKER_COLOR);
    }
    // //Print Order Seq
    for(int i=0;i<Pioneer::Move_Order.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_y(Pioneer::Move_Order[i].y),convert_world_pos_x(-Pioneer::Move_Order[i].x),map,ORDER_COLOR);
    }
    // //Print Robot
    Pioneer::draw_robot_at(convert_world_pos_y(ROBOT.y),convert_world_pos_x(-ROBOT.x),ROBOT.th,&map);
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
    for(int i=-40;i<-1;i++)
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
    ros::Rate rate(10);
    while(ros::ok())
    {
        mode_change=false;
        if(mode!=prev_mode)
        {
            ROS_INFO("MODE_CHANGE TO %d",mode);
            mode_change=true;
            prev_mode=mode;
        }
        run_camera();
        is_marker_on_sight();
        
        // Marker
        if(Marker_mode==MARKER_MODE::Position_adjust)
        {
            
        }
        else if(Marker_mode==MARKER_MODE::Complete_Marker)
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
            // adjust_th(cam_real_th);
            // adjust_x(cam_real_dis);
            //calculate middle pos
            //set to marker location
        }
        else if(Marker_mode==MARKER_MODE::No_Marker)
        {
            // Pioneer::stop();
            // ROS_INFO("STOP");
        }
    
        is_direction_match();
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
        // if(!start_stat||pause_stat||Arrive)
        // {
        //     mode=MODE::Stop;
        //     Pioneer::stop();
        // }
        cmd_vel_pub.publish(vel_msg);
        // mode=MODE::Stop;
        rate.sleep();
        ros::spinOnce();
    }
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
void Pioneer::pausecallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data=="stop")
    {
        ROS_INFO("Pioneer STOP");
        pause_stat=!pause_stat;  
    }
    else
    {
        ROS_INFO("Pioneer RESUME");
        //pause_stat=f;
    }  
}
void Pioneer::startcallback(const std_msgs::String::ConstPtr &msg)
{
   if(msg->data=="start")
    {
        ROS_INFO("Pioneer START");
        start_stat=true;  
    }
    else
    {
        start_stat=false;
    }
}
void Pioneer::odomcallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_msg=*msg;
    update_ROBOT_Position(msg);
    cout <<ROBOT.th<<endl;
}
void Pioneer::adjust_th(double th)
{
    // double time_duration=th/angle_speed;
    // while(adjust_end_time<=adjust_start_time+time_duration)
    // {
    // }
}
void Pioneer::adjust_x(double x)
{

}