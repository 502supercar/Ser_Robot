#ifndef _MOBILE_BASE_H
#define _MOBILE_BASE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string.h>

#define ROT_MAX_V 0.6      //the velocity of initial rotation
#define ROT_MIN_V 0.2
#define ROT_TOLERANT 0.2
#define PI 3.14159
#define ROT_ACC_LIMIT 0.5

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct mobile_goal
{
    double x;
    double y;
    double theta;
};

enum Footprint{OriginFootprint, SmallFootprint};

class Mobile_Base
{
private:
    ros::NodeHandle nh_;
    mobile_goal target_pose_;
    mobile_goal current_pose_;

    MoveBaseClient ac_;
    ros::Publisher car_tw_pub_;
    ros::Publisher car_fp_pub_;
    ros::Subscriber car_pose_sub_;

    void amclCbSetPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

public:
    Mobile_Base(ros::NodeHandle &nh);
    ~Mobile_Base();

    void getCurrentPose();
    bool carRotation(int angle);
    bool carGoStraight(double length, double speed);
    bool initRotation();
    void changeFootprint(Footprint fp);
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& move_base_feedback);
    void setTargetPose(mobile_goal goal);
    mobile_goal findObject();
    bool getGoalFromObject(double x, double y, double z);
    bool moveToGoal(std::string frame);
};

#endif