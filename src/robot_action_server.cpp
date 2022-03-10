#include "robot_control.hpp"
string order;
ros::Publisher start;
ros::Subscriber server_feedback;
ros::Subscriber server_result;
class Robot_Action
{
    protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot_action::robot_actionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    robot_action::robot_actionFeedback feedback_;
    robot_action::robot_actionResult result_;

    public:

    Robot_Action(std::string name):as_(nh_,name,boost::bind(&Robot_Action::executeCB,this,_1),false),action_name_(name)
    {
      
      as_.start();
    }
    ~Robot_Action(void)
    {}

    void executeCB(const robot_action::robot_actionGoalConstPtr &goal)
    {
      ros::Rate r(2);
      // as_.isPreemptRequested();
      // as_.setPreempted();
      order=goal->order;
      bool success=false;
      while(ros::ok())
      {
        if(as_.isNewGoalAvailable())
			    order=as_.acceptNewGoal()->order;
        if(success)
        {
        result_.res_Status="Arrive";
        ROS_INFO("SAFELY Arrirved at Destination");
        as_.setSucceeded(result_);
        break;
        }
        else
        {
          if(order=="START")
          {
            ROS_INFO("run");
            feedback_.curr_Status="aaa";
            as_.publishFeedback(feedback_);
            cout << order <<endl;
            //robot 위치 감지
          }
          else
          {
            ROS_INFO("waiting");
            feedback_.curr_Status="bbb";
            as_.publishFeedback(feedback_);
            cout << order <<endl;
          }
        }
         r.sleep();
      }
    }
    
};
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "robot_action_server");
  Robot_Action Robot_Action_("robot_action");
  ros::spin();

  return 0;
}