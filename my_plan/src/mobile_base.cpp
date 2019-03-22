#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <mobile_base.h>
#include <cmath>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool Mobile_Base::initRotation(ros::NodeHandle &node_handle)  //mobile_base initial rotation
{
    if(ros::ok())
    {
        //创建一个发布者，消息类型为geometry::Twist
	    ros::Publisher rotation_pub =node_handle.advertise<geometry_msgs::Twist>("/ir100_velocity_controller/cmd_vel",1000);

        //设置循环的频率
	    ros::Rate loop_rate(10);

        //初始化发送的消息变量
        geometry_msgs::Twist tw;

        //设置原点旋转的速度
        tw.linear.x=0;
	    tw.angular.z=INIT_ROT_ANG_V;

        float rot_duration=(4*PI)/INIT_ROT_ANG_V;   //3圈对应的弧度除以0.5rad/s
        int ticks=int(rot_duration*10);  //1一秒10次，算出计时次数

        ROS_INFO("Initial rotation started!!");
        for(int i = 0; i < ticks; i++)
        {
     	    //发布消息
		    rotation_pub.publish(tw);
            ros::spinOnce();
            loop_rate.sleep();
        }
	    ROS_INFO("Rotation completed!!");
        return true;
    }
    else
    {
        ROS_INFO("There is something wrong!");
        return false;
    }
}

void Mobile_Base::activeCb()
{
    ROS_INFO("Move_Base active!!!");
}

//move_base feedback callback function
void Mobile_Base::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& move_base_feedback)        
{
    //Get and save the current pose
    current_pose_.x = move_base_feedback->base_position.pose.position.x;
    current_pose_.y = move_base_feedback->base_position.pose.position.y;

    double z = move_base_feedback->base_position.pose.orientation.z;
    current_pose_.theta = 2*asin(z);

    //ROS_INFO("You are here: [x= %f,y= %f, theta= %f]",current_pose_.x,current_pose_.y,current_pose_.theta);
}

bool Mobile_Base::moveToGoal(ros::NodeHandle &node_handle)
{
    if(ros::ok())
    {
        //creat a move_base client
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        //RPY to Quaternion of the mobile_base
        goal.target_pose.pose.position.x = target_pose_.x;
        goal.target_pose.pose.position.y = target_pose_.y;
        goal.target_pose.pose.orientation.z = sin(target_pose_.theta/2);
        goal.target_pose.pose.orientation.w = cos(target_pose_.theta/2);

        ROS_INFO("Sending goal");
        ac.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(),
                        boost::bind(&Mobile_Base::activeCb, this),
                        boost::bind(&Mobile_Base::feedbackCb, this, _1));

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Mobile base has reached the target pose!");
            return true;
        }
        else
        {
            ROS_INFO("The mobile base failed to reach the target pose for some reason.");
            return false;
        }
    }
    else
    {
        ROS_INFO("There is something wrong!");
        return false;
    }
}

void Mobile_Base::getGoalFromObject(double x, double y, double z)
{
    //定义tf监听器
    tf::TransformListener listener;

    //定义存贮转换矩阵的变量
    tf::StampedTransform transform;

    try
    {
        //查找base与map的坐标转换
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    current_pose_.x = transform.getOrigin().x();
    current_pose_.y = transform.getOrigin().y();
    
    double angle = atan2((y - current_pose_.y),(x - current_pose_.x));
    if(y > current_pose_.y) target_pose_.y =y-0.8*sin(angle);
    else target_pose_.y=y+0.8*sin(angle);

    if(x > current_pose_.x) target_pose_.x=x-0.8*cos(angle);
    else target_pose_.x=x+0.8*cos(angle);

    target_pose_.theta = 0;

    ROS_INFO("Target:  x = %f, y = %f",target_pose_.x,target_pose_.y);
}
