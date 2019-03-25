#ifndef _MOBILE_BASE_H
#define _MOBILE_BASE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#define ROT_ANG_V 0.8      //the velocity of initial rotation
#define PI 3.14159

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct mobile_goal
{
    double x;
    double y;
    double theta;
};

class Mobile_Base
{
private:
    ros::NodeHandle nh_;
    mobile_goal target_pose_;
    mobile_goal current_pose_;

    MoveBaseClient ac_;
    ros::Publisher car_tw_pub_;
public:
    Mobile_Base(ros::NodeHandle &nh);
    ~Mobile_Base();

    void getCurrentPose();
    bool carRotation(int angle);
    bool initRotation();
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& move_base_feedback);
    void getGoalFromObject(double x, double y, double z);
    bool moveToGoal();
};

#endif
