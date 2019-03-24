#ifndef _MOBILE_BASE_H
#define _MOBILE_BASE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#define ROT_ANG_V 0.5      //the velocity of initial rotation
#define PI 3.14159


struct mobile_goal
{
    double x;
    double y;
    double theta;
};

class Mobile_Base
{
private:
    mobile_goal target_pose_;
    mobile_goal current_pose_;
public:
    void getCurrentPose();
    bool carRotation(int angle,ros::NodeHandle &node_handle);
    bool initRotation(ros::NodeHandle &node_handle);
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& move_base_feedback);
    bool moveToGoal(ros::NodeHandle &node_handle);
    void getGoalFromObject(double x, double y, double z);
};

#endif
