#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ser_robot/mobile_base.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>



//constructer
Mobile_Base::Mobile_Base(ros::NodeHandle &nh):nh_(nh),ac_("move_base", true),map()
{
    target_pose_.x = 0;
    target_pose_.y = 0;
    target_pose_.theta = 0;
    current_pose_.x = 0;
    current_pose_.y = 0;
    current_pose_.theta = 0;

    car_tw_pub_ = nh_.advertise<geometry_msgs::Twist>("/ir100_velocity_controller/cmd_vel",1000);
    car_fp_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/move_base/local_costmap/footprint",500);
    //car_fp_pub_ = nh_.advertise<geometry_msgs::Polygon>("/move_base/local_costmap/footprint",500);
    car_pose_sub_ = nh_.subscribe("/amcl_pose", 1000, &Mobile_Base::amclCbSetPose, this);
    map_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 250, &Mobile_Base::mapSubCb, this);
}

//disconsructer
Mobile_Base::~Mobile_Base()
{

}

//Get current pose by subscribing the amcl topic
void Mobile_Base::amclCbSetPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    double z = msg->pose.pose.orientation.z;
    current_pose_.theta = 2*asin(z);
    ROS_INFO("You are here: [x= %f,y= %f, theta= %f]",current_pose_.x,current_pose_.y,current_pose_.theta);
}

//Get local_costmap by subscribing /move_base/local_costmap/costmap
void Mobile_Base::mapSubCb(const nav_msgs::OccupancyGrid::ConstPtr& costmap)
{
    double resolution = costmap->info.resolution;
    unsigned int width = costmap->info.width;
    unsigned int height = costmap->info.height;
    double origin_x = costmap->info.origin.position.x;
    double origin_y = costmap->info.origin.position.y;
    
    //判断地图尺寸是否有变，变了就重新设置地图参数
    if(resolution != map.getResolution() || width != map.getSizeInCellsX() || height != map.getSizeInCellsY() ||
        origin_x != map.getOriginX() || origin_y != map.getOriginY())  map.resizeMap(width, height, resolution, origin_x, origin_y);

    unsigned int h,w;
    for(h = 0; h<height; h++)
    {
        for(w = 0; w<width; w++)
        {
            map.setCost(w, h, costmap->data[w+ width * h]);
        }
    }
    //map.saveMap("map");
}

//Get cost of one point in global_costmap or local_costmap
bool Mobile_Base::avoidCarBlock()
{
    double distance_origin = pow(target_pose_.y-current_pose_.y,2.0)+ pow(target_pose_.x - current_pose_.x, 2.0);
    double distance = pow(target_pose_.y - (current_pose_.y + 0.05*sin(current_pose_.theta)),2.0)+ pow(target_pose_.x - (current_pose_.x + 0.05*cos(current_pose_.theta)),2.0);
    if(distance_origin < distance)
    {
        carRotation(180);
        return true;
    }
    
    double x_forward = current_pose_.x + 0.5*cos(current_pose_.theta);
    double y_forward = current_pose_.y + 0.5*sin(current_pose_.theta);
    if(map.getCost((x_forward - map.getOriginX())/map.getResolution(), (y_forward - map.getOriginY())/map.getResolution()) == 0)
        carGoStraight(0.5,0.2);
    return true;
}

//Get current pose by TF
void Mobile_Base::getCurrentPose()
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
        ROS_ERROR("%s", ex.what());     //查找不到报错
        ros::Duration(1.0).sleep();
    }
    //把监听到的移动平台位置信息存入类的私有数据
    current_pose_.x = transform.getOrigin().x();
    current_pose_.y = transform.getOrigin().y();

    tf::Quaternion qua = transform.getRotation();   //获取四元数
    double roll,pitch,yaw;
    tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);    //把四元数转换为角度值
    current_pose_.theta = yaw;

    //ROS_INFO("Current: x= %f, y= %f,theta= %f",current_pose_.x,current_pose_.y,current_pose_.theta);
}

//function of car rotation
bool Mobile_Base::carRotation(int angle)
{
    //初始化发送的消息变量
    geometry_msgs::Twist tw;
    //设置原地旋转的速度
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    
    char rot_signal;
    if(angle > 0) rot_signal = 1;         //正的逆时针旋转
	else if(angle < 0) rot_signal = -1;   //负的顺时针旋转
    else return 0;

    getCurrentPose();       //获取当前小车的位姿
    //计算小车所需旋转的180度次数，以及剩下的度数
    int abs_angle = abs(angle);
    double start_offset = 0 - current_pose_.theta;      //设置相对零点
    int round = abs_angle/180;
    int remain_angle = abs_angle%180;
    double angle_rad = remain_angle/180.0 * PI;

    //执行180度的旋转
    double temp;
    int n = 0;
    double distance = 0;
    double vel = 0;
    while(round != 0)
    {
        //计算小车旋转180度后的位置
        if(((current_pose_.theta >0) && (angle >0)) || ((current_pose_.theta) >0 && (angle <0))) 
            temp = current_pose_.theta - PI;
        else temp = current_pose_.theta + PI;

        while(1)
        {
            distance = fabs(current_pose_.theta - temp);    //计算与目标相差的距离
            if(distance < ROT_TOLERANT) break;
            //ROS_INFO("current_pose_.theta= %f,temp= %f,distance= %f",current_pose_.theta,temp,distance);
            if(remain_angle == 0 && round == 1)
            {
                vel = sqrt(2*ROT_ACC_LIMIT*distance);
            }
            else vel = ROT_MAX_V;
            vel = std::min(std::max(vel,ROT_MIN_V),ROT_MAX_V);
            tw.angular.z = rot_signal*vel;
            car_tw_pub_.publish(tw);
            ros::spinOnce();    //等待回调函数，获取小车的位置
        }
       --round;
       ros::spinOnce();         //等待回调函数，获取小车的位置
    }

    //执行剩下角度的旋转
    double cur_angle = 0;
    distance=0;
    ros::spinOnce();            //等待回调函数，获取小车的位置

    start_offset = 0 - current_pose_.theta;
    while(remain_angle != 0)
    {
        cur_angle = current_pose_.theta + start_offset;
        distance = fabs(angle_rad-fabs(cur_angle));
        //ROS_INFO("cur_angle= %f,angle_rad= %f,distance= %f",cur_angle,angle_rad,distance);
        if(distance < ROT_TOLERANT) break;
        vel = sqrt(2*ROT_ACC_LIMIT*distance);
        vel = std::min(std::max(vel,ROT_MIN_V),ROT_MAX_V);
        tw.angular.z = rot_signal*vel;
        car_tw_pub_.publish(tw);
        ros::spinOnce();
    }

    tw.angular.z = 0.0;
    car_tw_pub_.publish(tw);
    ros::spinOnce();
    return 1;
}

bool Mobile_Base::carGoStraight(double length, double speed)
{
    //初始化发送的消息变量
    geometry_msgs::Twist tw;
    //设置原地旋转的速度
    tw.linear.y = 0.0;
    tw.angular.z = 0.0;

    char go_signal;
    if(length > 0) go_signal = 1;         //正的逆时针旋转
	else if(length < 0) go_signal = -1;   //负的顺时针旋转
    else return false;

    getCurrentPose();  //获取小车当前位姿
    mobile_pose target;
    target.x = length * cos(current_pose_.theta) + current_pose_.x;
    target.y = length * sin(current_pose_.theta) + current_pose_.y;

    double x_distance = 0, y_distance = 0;

    while(nh_.ok())
    {
        x_distance = fabs(current_pose_.x - target.x);
        y_distance = fabs(current_pose_.y - target.y);
        if(x_distance<0.12 && y_distance<0.12) break;
        tw.linear.x = go_signal * speed;
        car_tw_pub_.publish(tw);
        ros::spinOnce();
    }
    tw.linear.x = 0.0;
    car_tw_pub_.publish(tw);
    ros::spinOnce();
    return true;
}

bool Mobile_Base::initRotation()  //mobile_base initial rotation
{
    if(ros::ok())
    {
        ROS_INFO("Initial rotation started!!");
        carRotation(360);
	    ROS_INFO("Rotation completed!!");
        return true;
    }
    else
    {
        ROS_INFO("There is something wrong!");
        return false;
    }
}

//Change the mobile base footprint
void Mobile_Base::changeFootprint(Footprint fp)
{
    geometry_msgs::PolygonStamped footprint;
    //geometry_msgs::Polygon footprint;
    geometry_msgs::Point32 point[4];
    switch(fp)
    {
        case OriginFootprint:
        {
            //[[0.35, -0.26], [0.35, 0.26], [-0.35, 0.26], [-0.35, -0.26]]
            point[0].x = 0.35; point[0].y = -0.26;
            point[1].x = 0.35; point[1].y = 0.26;
            point[2].x = -0.35; point[2].y = 0.26;
            point[3].x = -0.35; point[3].y = -0.26;
            break;
        }
        case SmallFootprint:
        {
            point[0].x = 0.01; point[0].y = -0.01;
            point[1].x = 0.01; point[1].y = 0.01;
            point[2].x = -0.01; point[2].y = 0.01;
            point[3].x = -0.01; point[3].y = -0.01;
            break;
        }
    }
    footprint.header.stamp = ros::Time::now();
    footprint.header.frame_id = "base_link";
    footprint.polygon.points.push_back(point[0]);
    footprint.polygon.points.push_back(point[1]);
    footprint.polygon.points.push_back(point[2]);
    footprint.polygon.points.push_back(point[3]);
    car_fp_pub_.publish(footprint);
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

    /*actionlib::SimpleClientGoalState::StateEnum state;
    actionlib::SimpleClientGoalState car_state(state);
    car_state = ac_.getState();
    ROS_INFO("Car state is %s",car_state.toString().c_str());       //printf the move_base state*/

    //ROS_INFO("You are here: [x= %f,y= %f, theta= %f]",current_pose_.x,current_pose_.y,current_pose_.theta);
}

//Set private value target_pose_
void Mobile_Base::setTargetPose(mobile_pose goal)
{
    target_pose_.x = goal.x;
    target_pose_.y = goal.y;
    target_pose_.theta = goal.theta;
}

//巡航找到物体并返回物体相对于base_link的位置
Pos Mobile_Base::findObject()
{
    while(!ac_.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    //RPY to Quaternion of the mobile_base
    goal.target_pose.pose.position.x = -1.108;
    goal.target_pose.pose.position.y = 0.55;
    goal.target_pose.pose.orientation.z = 0.707;
    goal.target_pose.pose.orientation.w = 0.707;

    target_pose_.x = -1.108;
    target_pose_.y = 0.55;
    target_pose_.theta = 1.575;

    ROS_INFO("Sending goal");
    ac_.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(),
                    boost::bind(&Mobile_Base::activeCb, this),
                    boost::bind(&Mobile_Base::feedbackCb, this, _1));

    double x, y, x_aver=0, y_aver=0;
    double x_last_time=0;
    double x_get[10];
    double y_get[10];
    bool found_flag = false;
    char n_get=0;
    char n_maybe_get=0;

    mobile_pose ex_current_pose = {0,0,0};
    //定义tf监听器
    tf::TransformListener object_listener;

    //定义存贮转换矩阵的变量
    tf::StampedTransform object_transform;
    while(nh_.ok())
    {
        try
        {
            //查找object与base_link的坐标转换
            object_listener.lookupTransform("/zed_left_camera_frame", "/object", ros::Time(0), object_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());     //查找不到报错
            ros::Duration(1.0).sleep();
        }
        x = object_transform.getOrigin().x();
        y = object_transform.getOrigin().y();
        //ROS_INFO("x = %f,y = %f",x,y);
        if(x>0.001||x<-0.001||y>0.001||y<-0.001)
        {
            if(n_maybe_get>10)
            {
                ROS_INFO("Maybe find the object position!");
                
                ac_.cancelAllGoals();
                found_flag = true;
            }
            //ROS_INFO("n_maybe_get = %d,found_flag = %d",n_maybe_get,found_flag);
            switch(int(found_flag))
            {
                case 0:
                {
                    n_maybe_get++;
                    break;
                }
                case 1:
                {
                    try
                    {
                        //object_listener.waitForTransform("/base_link", "/object", ros::Time(0), ros::Duration(1.0));
                        object_listener.lookupTransform("/base_link", "/object", ros::Time(0), object_transform);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR("%s", ex.what());     //查找不到报错
                        ros::Duration(1.0).sleep();
                    }
                    if(std::isnan(object_transform.getOrigin().x()))    //判断是否是nan信号
                    {
                        continue;
                    }

                    if((object_transform.getOrigin().x() == x_last_time) || (fabs(object_transform.getOrigin().x()) > 2.0)) continue;         //判断相同的数据或异常数据

                    x_get[n_get] = object_transform.getOrigin().x();
                    y_get[n_get] = object_transform.getOrigin().y();
                    x_last_time = x_get[n_get];
                    ROS_INFO("x_get[%d] = %f,y_get[%d] = %f",n_get,x_get[n_get],n_get,y_get[n_get]);
                    n_get++;
                    if(n_get == 10)
                    {
                        char i;
                        for(i=0;i<10;i++)
                        {
                            x_aver = x_aver+x_get[i];
                            y_aver = y_aver+y_get[i];
                        }
                        x_aver = x_aver/10.0;
                        y_aver = y_aver/10.0;
                        for(i=0;i<10;i++)
                        {
                            if(fabs(x_get[i]-x_aver)>0.05)
                            {
                                found_flag =false;
                                n_get = 0;
                                x_aver = 0; y_aver = 0; x=0; y=0;
                                break;
                            }
                            if(i==9)
                            {
                                Pos target;
                                target.x = x_aver;
                                target.y = y_aver;
                                target.z = 0;
                                return target;
                            }
                        }
                    }
                    n_maybe_get = 0;
                    break;
                }
            }
        }
        else
        {
            n_get = 0;
            n_maybe_get=0;
            found_flag =false;
            x_aver = 0; y_aver = 0; x=0; y=0;
        }
        if((ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)&&(found_flag == false))
        {
            ROS_INFO("Mobile base has reached the target pose, but can not find object!");
        }
        else if(ac_.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("The mobile base failed to reach the target pose for some reason.");
        }
        else if((ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE) && (fabs(current_pose_.x - ex_current_pose.x) < 0.01) && 
            (fabs(current_pose_.theta - ex_current_pose.theta) < 0.01))
        {
            ROS_INFO("The mobile base was blocked! Action done!!.");
            avoidCarBlock();
        }
        ex_current_pose = current_pose_;
        ros::spinOnce();
    }
}

//Set the target of car by position of object
//x,y,z 都是相对于base_link  //object_pos相对base_link而言
mobile_pose Mobile_Base::getGoalFromObject(Pos object_pos)
{
    getCurrentPose();
    double x = object_pos.x;
    double y = object_pos.y;
    mobile_pose car_tar;
    double y_error,x_error;
    x_error = fabs(x);
    y_error = fabs(y);
    ROS_INFO(" x_error = %f, y_error = %f",x_error,y_error);
    if(x_error>0.7 || y_error>0.7)
    {
        double angle = atan2(y, x);
        if(y > 0) car_tar.y =y-0.7*sin(angle);
        else car_tar.y=y+0.7*sin(angle);

        if(x > 0) car_tar.x=x-0.7*cos(angle);
        else car_tar.x=x+0.7*cos(angle);

        car_tar.theta = angle;

        ROS_INFO("Target:  x = %f, y = %f",car_tar.x,car_tar.y);
        return car_tar;
    }
    else 
    {
        ROS_INFO("Robot can reach the object, do not need to go anywhere!");
        car_tar.x = 0;
        car_tar.y = 0;
        car_tar.theta = 0;
        return car_tar;
    }
}

//Start moving to the target
bool Mobile_Base::moveToGoal(mobile_pose tar_pose, std::string frame)
{
    if(ros::ok())
    {
        target_pose_= tar_pose;     //存储目标值

        //wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = frame;
        goal.target_pose.header.stamp = ros::Time::now();

        //RPY to Quaternion of the mobile_base
        goal.target_pose.pose.position.x = tar_pose.x;
        goal.target_pose.pose.position.y = tar_pose.y;
        goal.target_pose.pose.orientation.z = sin(tar_pose.theta/2);
        goal.target_pose.pose.orientation.w = cos(tar_pose.theta/2);

        ROS_INFO("Sending goal");
        ac_.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(),
                        boost::bind(&Mobile_Base::activeCb, this),
                        boost::bind(&Mobile_Base::feedbackCb, this, _1));

        ac_.waitForResult();

        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
