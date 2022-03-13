#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Char.h"
#include <time.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_action/robot_actionAction.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

using namespace std;

  cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 4.7717248258065513e+02, 0., 3.4345166112868361e+02, 0.,
                       4.7877303179533322e+02, 2.1840674677502400e+02, 0., 0., 1.);
  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -5.0679364100294742e-02, 1.9604492374058202e-01,
                        -8.2735393054366954e-04, -6.8036803338709421e-05,
                        -2.0917602681902503e-01); // Assuming no lens distortion

const double Marker_Gap=0.6;//meter
const double camera_width = 0.45;//meter
const double camera_length = 0.34;//meter
const double Robot_wheel_r = 0.11;//meter
const double Robot_center_to_camera_center = 0.42;//meter
int MARKER_FULL_num= 30000;

#define PI 3.141592
//ROBOT
ros::Publisher cmd_vel_pub;
ros::Subscriber key_input;
ros::Subscriber odom_status;//for testing

//SH_LEE
ros::Publisher robot_marker_dis_pub;//linear x value + linear y value angular z value(this option can be sent when most marker is shown)
ros::Publisher marker_on_sight;
ros::Publisher marker_center;

ros::Publisher marker_pass;//move robot linear 42cm if we have to rotate send true once
ros::Publisher pos_spin_prev;//send true before spin
ros::Publisher pos_spin_after;//send true after spin

ros::Subscriber correction; //type std_msgs bool
ros::Subscriber camera_pos;
ros::Subscriber robot_pos;

//SERVER
ros::Subscriber stop_status;

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
    ros::Time compare_time_1;
    ros::Time compare_time_2;
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
    bool MARKER_pub_once;
    bool Arrive;
    struct COLOR MARKER_COLOR;
    struct COLOR ROBOT_COLOR_X;
    struct COLOR ROBOT_COLOR_Y;
    struct COLOR ORDER_COLOR;
    struct COLOR PATH_COLOR;

    struct POSITION ROBOT_POS;
    struct Visual_Map Map;
    geometry_msgs::Twist vel_msg;
    nav_msgs::Odometry odom_msg;

    int count;
    int test_count;
    int PATH_index;
    int Marker_index;
    int marker_num;
    int order_num;
    POSITION ROBOT;
    vector<POSITION> Move_Order;
    vector<POSITION> MARKER;
    vector<POSITION> PATH;
    cv::VideoCapture capture;
    
    //SH_LEE
    POSITION receive_camera_pos;
    POSITION receive_robot_pos;
    bool receive_correction;

    //ACTION//
    bool pause_stat;
    bool start_stat;
    public:
    Pioneer();
    ~Pioneer();
    //Robot Movement
    void Get_param();
    void set_Position(struct POSITION &pos,double x,double y,double th);
    void add_Position(struct POSITION &pos,double x,double y,double th);
    bool set_mode(int num);
    void set_cmd_vel(double x,double th);
    void go_front();
    void turn_left();
    void turn_right();
    void stop();
    void back();
    void run_robot();
    bool update_ROBOT_Position(const nav_msgs::Odometry::ConstPtr &msg);
    void add_path_or_marker(vector<POSITION> &Pos, double x,double y,int order_num);

    void calc_adjust_val();//When Marker is 15% shown do adjust_th first then adjust_x
    void exact_adjust_val();//When Marker is 90% shown do adjust_x first then adjust_th
    void adjust_th(double th);
    void adjust_x(double x);
    void is_direction_match();
    void is_marker_match();
    void is_destination_arrive();
    void pub_geometry_twist_val(geometry_msgs::Twist& val);

    void keycallback(const std_msgs::Char::ConstPtr &msg);
    void pausecallback(const std_msgs::String::ConstPtr &msg);
    void startcallback(const std_msgs::String::ConstPtr &msg);
    void odomcallback(const nav_msgs::Odometry::ConstPtr &msg);

    void correctioncallback(const std_msgs::Bool::ConstPtr &msg);
    void camera_poscallback(const geometry_msgs::Pose::ConstPtr &msg);
    void robot_poscallback(const geometry_msgs::Pose::ConstPtr &msg);
    //Camera Part
    void is_marker_on_sight();
    bool Publish_image();
    bool run_camera();
    bool determine_color(cv::Mat& frame,int B,int G,int R,int col,int row);
    //visualize
    void set_Visual_map(int cross,int row,int col);
    void set_color(struct COLOR &color,int R,int G,int B,int offset);
    void visualize();
    void draw_robot_at(double x,double y,double th,cv::Mat *map);
    void draw_marker_at(double x,double y,cv::Mat &map,struct COLOR color);
    void draw_path_at(double x,double y,cv::Mat &map,struct COLOR color);
    int convert_world_pos_x(double x);
    int convert_world_pos_y(double y);
};
  
Pioneer::Pioneer()
{ 
    mode=MODE::Stop;
    pause_stat=false;
    start_stat=false;
    prev_mode=MODE::Stop;
    test_count=0;
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
		std::cerr<<"Camera open failed!"<<std::endl;
    else
        ROS_INFO("Camera model [ELP-USBFHD06H-L21] connected");
    count=0;
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
    set_color(PATH_COLOR,10,225,10,0);
    set_Position(ROBOT,0,0,0);
    set_Visual_map(14,700,700);
    order_num=0;
    Marker_index=0;
    MARKER_pub_once=false;
    Arrive=false;
    set_Position(receive_robot_pos,0,0,0);
    set_Position(receive_camera_pos,0,0,0);
    receive_correction=false;
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
    //SH_LEE
    marker_on_sight=nh_.advertise<std_msgs::Bool>("Marker_onsight",1);
    marker_center=nh_.advertise<std_msgs::Bool>("Marker_center",1);
    marker_pass=nh_.advertise<std_msgs::Bool>("Marker_pass",1);
    
    pos_spin_prev=nh_.advertise<std_msgs::Bool>("pos_spin_prev",1);
    pos_spin_after=nh_.advertise<std_msgs::Bool>("pos_spin_after",1);

    robot_marker_dis_pub=nh_.advertise<geometry_msgs::Twist>("Marker_distance",10);

    correction=nh_.subscribe("/chk_goal",1,&Pioneer::correctioncallback,this);
    camera_pos=nh_.subscribe("/kalman_mean",10,&Pioneer::camera_poscallback,this);
    robot_pos=nh_.subscribe("/robot_pos",10,&Pioneer::robot_poscallback,this);

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
        y=atoi(marker_pos.substr(index+1).c_str());
        marker_pos.erase(index,marker_pos.size()-1);
        x=atoi(marker_pos.c_str());
        add_path_or_marker(MARKER,Marker_Gap*x,Marker_Gap*y,i);
        
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
        y=atoi(order_pos.substr(index+1).c_str());
        order_pos.erase(index,order_pos.size()-1);
        x=atoi(order_pos.c_str());
        add_path_or_marker(Move_Order,Marker_Gap*x,Marker_Gap*y,i);
    }
    ROS_INFO("%d ORDER_SET",order_num);
}
void Pioneer::set_Position(struct POSITION &pos,double x,double y,double th)
{
    pos.x=x;
    pos.y=y;
    pos.th=th;
}
void Pioneer::add_Position(struct POSITION &pos,double x,double y,double th)
{
    pos.x+=x;
    pos.y+=y;
    pos.th+=th;
    ROS_INFO("ROBOT_x: %f ROBOT_y: %f ROBOT_th: %f",pos.x,pos.y,pos.th);
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
void Pioneer::add_path_or_marker(vector<POSITION> &Pos, double x,double y,int order_num)
{
    POSITION temp;
    set_Position(temp,x,y,0);
    temp.order_num=order_num;
    Pos.push_back(temp);
}
void Pioneer::is_marker_on_sight()//90% of marker is shown
{
    if(Marker_mode==MARKER_MODE::Complete_Marker)
    {
        if(MARKER_pixel<MARKER_FULL_num*0.1)
        {
            Marker_mode=MARKER_MODE::No_Marker;
            MARKER_pub_once=false;   
        }
    }
    else
    {
        if(MARKER_pixel>=MARKER_FULL_num*0.9)//90% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
     
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.85&&Marker_mode==MARKER_MODE::Goto_Marker)//hysteresis - Goto
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.15)//15% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Find_Marker;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.1&&Marker_mode==MARKER_MODE::Find_Marker)//hysteresis - Find
    {
        Marker_mode=MARKER_MODE::Find_Marker;
    }
    else
    {
        Marker_mode=MARKER_MODE::No_Marker;
    }

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
    //  Pioneer::visualize();
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
        
        // ROS_INFO("X_POS = %d",int(x_pos));
        // ROS_INFO("Y_POS = %d",int(y_pos));
        cam_pixel_x_pos=x_pos;
        cam_pixel_y_pos=y_pos;
        cam_real_y_pos=400-cam_pixel_x_pos;
        cam_real_x_pos=300-cam_pixel_y_pos;

        cam_real_x_pos/=1754.4;
        cam_real_y_pos/=1754.4;
        cam_real_dis=sqrt(cam_real_x_pos*cam_real_x_pos+cam_real_y_pos*cam_real_y_pos);
        // cam_real_x_pos+=Robot_center_to_camera_center;
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
                    // int H=mod_frame.at<cv::Vec3b>(i,j)[0];
                    // int S=mod_frame.at<cv::Vec3b>(i,j)[1];
                    // int V=mod_frame.at<cv::Vec3b>(i,j)[2];
                    if(j>Max_x.x&&determine_color(frame,0,0,255,j-1,i)&&determine_color(frame,0,0,255,j-2,i))
                        set_Position(Max_x,j,i,0);
                    if(j<Min_x.x&&determine_color(frame,0,0,255,j+1,i)&&determine_color(frame,0,0,255,j+2,i))
                        set_Position(Min_x,j,i,0);
                    if(i>Max_y.y&&determine_color(frame,0,0,255,j,i-1)&&determine_color(frame,0,0,255,j,i-2))
                        set_Position(Max_y,j,i,0);
                    if(i<Min_y.y&&determine_color(frame,0,0,255,j,i+1)&&determine_color(frame,0,0,255,j,i+2))
                        set_Position(Min_y,j,i,0);   
                }
            }
            double dis1=sqrt(pow((cam_pixel_x_pos-Max_x.x),2)+pow((cam_pixel_y_pos-Max_x.y),2));
            double dis2=sqrt(pow((cam_pixel_x_pos-Min_x.x),2)+pow((cam_pixel_y_pos-Min_x.y),2));
            double dis3=sqrt(pow((cam_pixel_x_pos-Max_y.x),2)+pow((cam_pixel_y_pos-Max_y.y),2));
            double dis4=sqrt(pow((cam_pixel_x_pos-Min_y.x),2)+pow((cam_pixel_y_pos-Min_y.y),2));
            if(dis1<125&&dis2<125&&dis3<125&&dis4<125)
            {
                if(dis1>115&&dis2>115&&dis3>115&&dis4>115)
                {
                    double left_th=atan((Max_x.x-Max_y.x)/(Max_y.y-Max_x.y));
                    double right_th=atan((Min_x.x-Max_y.x)/(Min_y.y-Max_x.y));
                    // cout <<"Left_th: "<<left_th<<endl;
                    // cout <<"Right_th: "<<right_th<<endl;
                    if(left_th>right_th)
                    {
                        robot_adjust_th=-(PI/4-right_th);
                    }
                    else
                    {
                        robot_adjust_th=(PI/4-left_th);
                    }
                    //계산
                }
                else
                {
                    robot_adjust_th=PI/4;
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
            if(sqrt((cam_pixel_x_pos-400)*(cam_pixel_x_pos-400)+(cam_pixel_y_pos-300)*(cam_pixel_y_pos-300))<2500)
            Marker_mode=MARKER_MODE::Complete_Marker;
            // cout <<abs(cam_pixel_x_pos-400)<<endl;
            // cout <<abs(cam_pixel_y_pos-300)<<endl;
            
        }
        
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
                // if(110>=H&&H>=90&&S<145&&V<240&&V>205)
                // {
                //     frame.at<cv::Vec3b>(i,j)={0,255,0};
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
    for(int i=0;i<Pioneer::PATH.size();i++)
    {
        Pioneer::draw_path_at(convert_world_pos_y(PATH[i].x),convert_world_pos_x(-PATH[i].y),map,PATH_COLOR);
    }
    // Print Marker(whick is found)
    for(int i=0;i<Pioneer::MARKER.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_x(-Pioneer::MARKER[i].y),convert_world_pos_y(Pioneer::MARKER[i].x),map,MARKER_COLOR);
    }
    // //Print Order Seq
    for(int i=0;i<Pioneer::Move_Order.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_x(-Pioneer::Move_Order[i].y),convert_world_pos_y(Pioneer::Move_Order[i].x),map,ORDER_COLOR);
    }
    
    // //Print Robot
    Pioneer::draw_robot_at(convert_world_pos_y(ROBOT.x),convert_world_pos_x(-ROBOT.y),ROBOT.th,&map);
    cv::namedWindow("map");
    cv::moveWindow("map",865,0);
    cv::imshow("map",map);
    cv::waitKey(10)==27;
}
void Pioneer::draw_robot_at(double x,double y,double th,cv::Mat *map)
{
    double cos_th=cos(ROBOT.th);
    double sin_th=sin(ROBOT.th);
    
    for(int i=-30;i<-4;i++)
    {
        for(int j=-3;j<=3;j++)
        {
            int row=int(i*cos_th+j*sin_th+x);
            int col=int(i*sin_th-j*cos_th+y);
            if(row<0)
                row=0;
            if(row>map->rows)
                row=map->rows-1;
            if(col<0)
                col=0;
            if(col>map->cols)
                col=map->cols-1;
            map->at<cv::Vec3b>(row,col)={ROBOT_COLOR_X.B,ROBOT_COLOR_X.G,ROBOT_COLOR_X.R};
        }
    }
    for(int i=-30;i<4;i++)
    {
        for(int j=-3;j<=3;j++)
        {   
            int row=int(-i*sin_th-j*cos_th+x);
            int col=int(i*cos_th-j*sin_th+y);
            if(row<0)
                row=0;
            if(row>map->rows)
                row=map->rows-1;
            if(col<0)
                col=0;
            if(col>map->cols)
                col=map->cols-1;
            map->at<cv::Vec3b>(row,col)={ROBOT_COLOR_Y.B,ROBOT_COLOR_Y.G,ROBOT_COLOR_Y.R};    
        }
    }
}
void Pioneer::draw_marker_at(double x,double y,cv::Mat &map,struct COLOR color)
{
    for(int i=-5;i<=5;i++)
    {
        for(int j=-5;j<=5;j++)
        {   
        map.at<cv::Vec3b>(i+y,j+x)={color.B,color.G,color.R};
        }
    }
}
void Pioneer::draw_path_at(double x,double y,cv::Mat &map,struct COLOR color)
{
    for(int i=-1;i<=1;i++)
    {
        for(int j=-1;j<=1;j++)
        {
            
            int row=int(i+x);
            int col=int(j+y);
            // if(row>0&&row<map.rows-1&&col>0&&col<map.cols-1)
                map.at<cv::Vec3b>(row,col)={color.B,color.G,color.R};
        }
    }
}
int Pioneer::convert_world_pos_x(double x)
{
    double world_x=Map.row_block*(x*(1.0/Marker_Gap)+1);
    return int(world_x);
}
int Pioneer::convert_world_pos_y(double y)
{
    double world_y=Map.col_block*(double(Map.cross-1)-y*(1.0/Marker_Gap));
    return int(world_y);
}
void Pioneer::is_direction_match()
{
    /*atan version*/
    double temp_x=MARKER[Marker_index].x-ROBOT.x;
    double temp_y=MARKER[Marker_index].y-ROBOT.y;
    // cout <<Marker_index<<endl;
    // cout <<"x: "<<MARKER[0].x<<endl;
    // cout <<"y: "<<MARKER[0].y<<endl;
    
    double temp_th=atan(temp_y/(temp_x));
    // double temp_th_1=acos(temp_x/sqrt((temp_x*temp_x+temp_y*temp_y)))-ROBOT.th;
    // double temp_th_2=ROBOT.th-acos(temp_x/sqrt((temp_x*temp_x+temp_y*temp_y)));
    // cout <<"temp_x: "<<temp_x<<endl;
    // cout <<"temp_y: "<<temp_y<<endl;
    
    // if(temp_x<0.1)
    // {
    //     if(temp_y<0)
    //     {
    //         temp_th=PI/2;
    //     }
    //     else
    //     {
    //         temp_th=-PI/2;
    //     }
    // }
    
    // cout<<temp_th<<endl;
    temp_th+=ROBOT.th;
    test_count++;
        if(test_count==10)
        {
            cout<<temp_th<<endl;
            test_count=0;
        }
    // cout <<temp_th<<endl;
    
    if(abs(temp_th)<0.15)
    {
        // ROS_INFO("Direction Matched");
        mode=MODE::Front;
    }
    else
    {
        if(temp_th<0)
        {
            ROS_INFO("turn left");
            mode=MODE::Left;
        }
        else
        {
            ROS_INFO("turn right");
            mode=MODE::Right;
        }
    }

    // double Rob_cos=cos(ROBOT.th);
    // double Rob_sin=sin(ROBOT.th);
    
    // double th=PI/2-acos(temp_x/sqrt(temp_x*temp_x+temp_y+temp_y));
    // double Mar_cos=cos(th);
    // double Mar_sin=sin(th);
    
    // double determine=Rob_cos*Mar_cos+Rob_sin*Mar_sin;
    // cout <<th<<endl;
    // cout << determine<<endl;
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
        geometry_msgs::Twist marker_pose_temp;
        std_msgs::Bool TRUE_msgs;
        std_msgs::Bool FALSE_msgs;
        FALSE_msgs.data=false;
        TRUE_msgs.data=true;
        count++;
        if(count==10)
        {
            count=0;
            PATH_index%=100;
            
            if(PATH.size()<100)
            PATH.push_back(ROBOT);
            else
            {
                PATH[PATH_index]=ROBOT;
            }
        }
        mode_change=false;
        if(mode!=prev_mode)
        {
            ROS_INFO("MODE_CHANGE TO %d",mode);
            mode_change=true;
            prev_mode=mode;
        }
        run_camera();
        visualize();
        cv::waitKey(10)==27;
        is_marker_on_sight();
        
        marker_pass.publish(FALSE_msgs);
        // Marker
        if(Marker_mode==MARKER_MODE::Goto_Marker)
        {
            marker_on_sight.publish(TRUE_msgs);
        }
        else
        {
            marker_on_sight.publish(FALSE_msgs);
        }
        
        if(Marker_mode==MARKER_MODE::Position_adjust)
        { 
        }
        else if(Marker_mode==MARKER_MODE::Complete_Marker&&!MARKER_pub_once)//
        {
            pub_geometry_twist_val(marker_pose_temp);
            marker_center.publish(TRUE_msgs);
            MARKER_pub_once=true;
        }
        else if(Marker_mode==MARKER_MODE::Goto_Marker)
        {
            pub_geometry_twist_val(marker_pose_temp);
            ROS_INFO("SEE MOST MARKER");
        }
        else if(Marker_mode==MARKER_MODE::Find_Marker)
        {
            pub_geometry_twist_val(marker_pose_temp);
            ROS_INFO("SEE MARKER");
        }
        else if(Marker_mode==MARKER_MODE::No_Marker)
        {
    
        }
        is_direction_match();
        is_marker_match();
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
    if(msg->data=='W'||msg->data=='w')
    {
        mode=MODE::Front;
        ROS_INFO("CHANGE_MODE -> FRONT");
        //testing purpose
        add_Position(ROBOT,0.02*cos(ROBOT.th),+0.02*sin(ROBOT.th),0);
    }
    else if(msg->data=='S'||msg->data=='s')
    {
        mode=MODE::Stop;
        ROS_INFO("CHANGE_MODE -> STOP");
        //testing purpose
    }
    else if(msg->data=='D'||msg->data=='d')
    {
        mode=MODE::Right;
        ROS_INFO("CHANGE_MODE -> RIGHT");
        add_Position(ROBOT,0,0,-0.05);
        //testing purpose
    }
    else if(msg->data=='A'||msg->data=='a')
    {
        mode=MODE::Left;
        ROS_INFO("CHANGE_MODE -> LEFT");
        add_Position(ROBOT,0,0,0.05);
        //testing purpose
    }
    else if(msg->data=='X'||msg->data=='x')
    {
        mode=MODE::Back;
        ROS_INFO("CHANGE_MODE -> BACK");
        add_Position(ROBOT,-0.1*cos(ROBOT.th),-0.1*sin(ROBOT.th),0);
        //testing purpose
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
    // cout <<ROBOT.th<<endl;
}
void Pioneer::correctioncallback(const std_msgs::Bool::ConstPtr &msg)
{
    receive_correction=msg->data;
}
void Pioneer::camera_poscallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    receive_camera_pos.x=msg->position.x;
    receive_camera_pos.y=msg->position.y;
    tf::Pose pose;
    tf::poseMsgToTF(*msg,pose);
    //Turn Quaternion to Euler
    receive_camera_pos.th=tf::getYaw(pose.getRotation());
}
void Pioneer::robot_poscallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    receive_robot_pos.x=msg->position.x;
    receive_robot_pos.y=msg->position.y;
    tf::Pose pose;
    tf::poseMsgToTF(*msg,pose);
    //Turn Quaternion to Euler
    receive_robot_pos.th=tf::getYaw(pose.getRotation());
    ROBOT.x=receive_robot_pos.x;
    ROBOT.y=receive_robot_pos.y;
    ROBOT.th=receive_robot_pos.th;
}
void Pioneer::adjust_th(double th)
{
    double time_duration=abs(th)/angle_speed;
    
    compare_time_1=ros::Time::now();
    compare_time_2=ros::Time::now();
    ROS_INFO("%f",time_duration);
    while(compare_time_1.sec+time_duration>=compare_time_2.sec)
    {   
        compare_time_2=ros::Time::now();
        if(th>0.1)
        {
            Pioneer::turn_left();
        }
        else if(th<-0.1)
        {
            Pioneer::turn_right();
        }
        else
        {
            break;
        }
        cmd_vel_pub.publish(vel_msg);
    }
}
void Pioneer::adjust_x(double x)
{
    double time_duration=abs(x)/speed;
    
    compare_time_1=ros::Time::now();
    compare_time_2=ros::Time::now();
    ROS_INFO("%f",time_duration);
    while(compare_time_1.sec+time_duration>=compare_time_2.sec)
    {   
        compare_time_2=ros::Time::now();
        if(x>0.05)
        {
            Pioneer::go_front();
        }
        else if(x<-0.05)
        {
            Pioneer::back();
        }
        else
        {
            break;
        }
        cmd_vel_pub.publish(vel_msg);
    }
}
bool Pioneer::determine_color(cv::Mat& frame,int B,int G,int R,int col,int row)
{
    int temp_B,temp_G,temp_R;
    temp_B=frame.at<cv::Vec3b>(row,col)[0];
    temp_G=frame.at<cv::Vec3b>(row,col)[1];
    temp_R=frame.at<cv::Vec3b>(row,col)[2];
    if(temp_B==B&&temp_G==G&&temp_R==R)
        return true;
    else 
        return false;
}
void Pioneer::pub_geometry_twist_val(geometry_msgs::Twist& val)
{
    val.linear.x=cam_real_x_pos;
    val.linear.y=cam_real_y_pos;
    if(Marker_mode>MARKER_MODE::Find_Marker)
    val.angular.z=robot_adjust_th;
}
void Pioneer::is_marker_match()
{
    if(abs(MARKER[Marker_index].x-ROBOT.x)<0.2&&abs(MARKER[Marker_index].y)<0.2)
    {
        if(Marker_index==marker_num)
        {
            ROS_INFO("arrive %d",Marker_index);
            Arrive=true;
            mode=MODE::Stop;
        }
        else
        {
            ROS_INFO("Passed MARKER %d Heading to MARKER %d",Marker_index,Marker_index+1);
            Marker_index++;
        }
    }
}