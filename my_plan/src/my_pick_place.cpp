#include "my_pick_place.h"
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
//#include <unistd.h>
//#include <mobile_base.h>
#include <cmath>

const double FINGER_MAX=6400;
int mode;

using namespace kinova;

//ZYZ欧拉角转化为四元数
tf::Quaternion EulerZYZ_to_Quaternion(double tz1,double ty,double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();   //初始化为单位矩阵

    //矩阵右乘
    rot_temp.setEulerYPR(tz1,0.0,0.0);
    rot*=rot_temp;
    rot_temp.setEulerYPR(0.0,ty,0.0);
    rot*=rot_temp;
    rot_temp.setEulerYPR(tz2,0.0,0.0);
    rot*=rot_temp;

    rot.getRotation(q);
    return q;
}

PickPlace::PickPlace(ros::NodeHandle &nh):nh_(nh)
{
    //读取参数
    nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300");
    nh_.param<bool>("/robot_connected",robot_connected_,true);
    if(robot_connected_)
    {
        //订阅话题，moveit才能进行
        sub_pose_=nh_.subscribe<geometry_msgs::PoseStamped>("/"+robot_type_+"_driver/out/tool_pose",1,&PickPlace::get_current_pose,this);
    }

    //初始化类接口
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_=robot_model_loader.getModel();

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    visual_tools_=new moveit_visual_tools::MoveItVisualTools("base_link");

    arm_group_=new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_=new moveit::planning_interface::MoveGroupInterface("gripper");

    //robot_state::RobotState& robot_state_=planning_scene_->getCurrentStateNonConst();
    robot_state_=arm_group_->getCurrentState();
    joint_model_group_=robot_state_->getJointModelGroup("arm");

    arm_group_->setEndEffectorLink(robot_type_+"_end_effector");

    finger_client_=new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>("/"+robot_type_+"_driver/fingers_action/finger_positions",false);
    while(robot_connected_&&!finger_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the finger action server to come up");
    }

    //注册发布话题
    pub_co_=nh_.advertise<moveit_msgs::CollisionObject>("/collision_object",10);
    pub_aco_=nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object",10);
    pub_planning_scene_diff_=nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene",1);
    //pub_trajectory_=nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1);

    //初始化joint_name_,joint_value_
    int arm_joint_num=robot_type_[3]-'0';
    joint_name_.resize(arm_joint_num);
    joint_value_.resize(arm_joint_num);
    for(int i=0;i<joint_name_.size();i++)
    {
        joint_name_[i]=robot_type_+"_joint_"+boost::lexical_cast<std::string>(i+1);
    }

}


//停止发布节点及删除堆空间
PickPlace::~PickPlace()
{
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();
    //pub_trajectory_.shutdown();

    delete arm_group_;
    delete gripper_group_;
    delete finger_client_;
    delete visual_tools_;
}


//回调函数，接收current_pose_
void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_=*msg;
}

//回调函数，接收current_state_
void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_=*msg;
}


void PickPlace::add_obstacle()
{
    clear_obstacle();

    co_.id="pole";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type=shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(3);
    co_.primitives[0].dimensions[0]=0.3;
    co_.primitives[0].dimensions[1]=0.1;
    co_.primitives[0].dimensions[2]=0.4;
    co_.primitive_poses[0].position.x=0.5;
    co_.primitive_poses[0].position.y=-0.1;
    co_.primitive_poses[0].position.z=0.4/2.0;

    co_.operation=moveit_msgs::CollisionObject::ADD;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff=true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();

}

void PickPlace::clear_obstacle()
{
    co_.id="pole";
    co_.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff=true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();

}

void PickPlace::add_attached_obstacle()
{
    co_.id="target_cylinder";
    co_.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    aco_.object.operation=moveit_msgs::CollisionObject::ADD;
    aco_.link_name=robot_type_+"_end_effector";
    aco_.touch_links.push_back(robot_type_+"_end_effector");
    aco_.touch_links.push_back(robot_type_+"_link_finger_1");
    aco_.touch_links.push_back(robot_type_+"_link_finger_2");
    aco_.touch_links.push_back(robot_type_+"_link_finger_3");
    aco_.touch_links.push_back(robot_type_+"_link_finger_tip_1");
    aco_.touch_links.push_back(robot_type_+"_link_finger_tip_2");
    aco_.touch_links.push_back(robot_type_+"_link_finger_tip_3");
    pub_aco_.publish(aco_);
}

void PickPlace::remove_attached_obstacle()
{
    aco_.object.id="target_cylinder";
    aco_.object.operation=moveit_msgs::CollisionObject::REMOVE;
    aco_.touch_links.clear();
    pub_aco_.publish(aco_);
}

void PickPlace::add_target(double radius,double height,double x,double y,double z)
{
    co_.id="target_cylinder";
    co_.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type=shape_msgs::SolidPrimitive::CYLINDER;
    co_.primitives[0].dimensions.resize(2);

    co_.primitives[0].dimensions[0]=radius;
    co_.primitives[0].dimensions[1]=height;
    co_.primitive_poses[0].position.x=x;
    co_.primitive_poses[0].position.y=y;
    co_.primitive_poses[0].position.z=z;

    co_.operation=moveit_msgs::CollisionObject::ADD;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff=true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    aco_.object=co_;
    ros::WallDuration(0.1).sleep();
}

void PickPlace::build_workscene()
{
    co_.header.frame_id="base_link";
    co_.header.stamp=ros::Time::now();

    co_.id="table";
    co_.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type=shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(3);
    co_.primitives[0].dimensions[0]=2.4;
    co_.primitives[0].dimensions[1]=2.4;
    co_.primitives[0].dimensions[2]=0.1;
    co_.primitive_poses[0].position.x=0.0;
    co_.primitive_poses[0].position.y=0.0;
    co_.primitive_poses[0].position.z=-0.01;

    co_.operation=moveit_msgs::CollisionObject::ADD;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff=true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
}

void PickPlace::clear_workscene()
{
    co_.id="table";
    co_.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    co_.id="target_cylinder";
    co_.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    aco_.object.operation=moveit_msgs::CollisionObject::REMOVE;
    pub_aco_.publish(aco_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff=true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);

    clear_obstacle();
    remove_attached_obstacle();

}

void PickPlace::display_trajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{

    visual_tools_->deleteAllMarkers();
    //display_trajectory_.trajectory_start=plan.start_state_;
    //display_trajectory_.trajectory.push_back(plan.trajectory_);
    //pub_trajectory_.publish(display_trajectory_);
    visual_tools_->publishTrajectoryLine(plan.trajectory_,joint_model_group_);
    visual_tools_->trigger();
    ros::WallDuration(1.0).sleep();
}

//给定start_pose_,grasp_pose_
void PickPlace::define_grasp_cartesian_pose(double x,double y,double z)
{
    tf::Quaternion q;
    /*start_pose_.header.frame_id="base_link";
    start_pose_.header.stamp=ros::Time::now();
    start_pose_.pose.position.x=0.5;
    start_pose_.pose.position.y=-0.5;
    start_pose_.pose.position.z=0.5;

    q=EulerZYZ_to_Quaternion(-M_PI/4,M_PI/2,M_PI/2);
    start_pose_.pose.orientation.x=q.x();
    start_pose_.pose.orientation.y=q.y();
    start_pose_.pose.orientation.z=q.z();
    start_pose_.pose.orientation.w=q.w();*/

    grasp_pose_.header.frame_id="base_link";
    grasp_pose_.header.stamp=ros::Time::now();
    /*
    grasp_pose_.pose.position.x=0.6;
    grasp_pose_.pose.position.y=0.6;
    grasp_pose_.pose.position.z=0.3;
    */

    grasp_pose_.pose.position.x=x;
    grasp_pose_.pose.position.y=y;
    grasp_pose_.pose.position.z=z;

    q=EulerZYZ_to_Quaternion(M_PI/4,M_PI/2,M_PI/2);
    grasp_pose_.pose.orientation.x=q.x();
    grasp_pose_.pose.orientation.y=q.y();
    grasp_pose_.pose.orientation.z=q.z();
    grasp_pose_.pose.orientation.w=q.w();

    grasp_pose_=generate_gripper_align_pose(grasp_pose_,0.04,M_PI/4,M_PI/2,M_PI/2);
    pregrasp_pose_=generate_gripper_align_pose(grasp_pose_,0.1,M_PI/4,M_PI/2,M_PI/2);
    postgrasp_pose_=grasp_pose_;
    postgrasp_pose_.pose.position.z+=0.05;
}

void PickPlace::define_place_cartesian_pose(double x,double y,double z)
{
    tf::Quaternion q;

    place_pose_.header.frame_id="base_link";
    place_pose_.header.stamp=ros::Time::now();
   /*
    place_pose_.pose.position.x=0.5;
    place_pose_.pose.position.y=-0.5;
    place_pose_.pose.position.z=0.45;
  */

    place_pose_.pose.position.x=x;
    place_pose_.pose.position.y=y;
    place_pose_.pose.position.z=z;

    q=EulerZYZ_to_Quaternion(-M_PI/4,M_PI/2,M_PI/2);
    place_pose_.pose.orientation.x=q.x();
    place_pose_.pose.orientation.y=q.y();
    place_pose_.pose.orientation.z=q.z();
    place_pose_.pose.orientation.w=q.w();

    preplace_pose_=place_pose_;
    preplace_pose_.pose.position.z+=0.05;
    place_pose_.pose.position.z+=0.01;
    postplace_pose_=generate_gripper_align_pose(place_pose_,0.07,-M_PI/4,M_PI/2,M_PI/2);
}

//产生离姿态targetpose_msg距离为dist的新姿态,azimuth为在XY平面上与X轴的夹角，polar为与Z轴的夹角
//azimuth,polar,rot_gripper_z为定义姿态的ZYZ欧拉角
geometry_msgs::PoseStamped PickPlace::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg,double dist,double azimuth,double polar,double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id="base_link";

    tf::Quaternion q=EulerZYZ_to_Quaternion(azimuth,polar,rot_gripper_z);
    double x_dist=-dist*cos(azimuth)*sin(polar);
    double y_dist=-dist*sin(azimuth)*sin(polar);
    double z_dist=-dist*cos(polar);

    pose_msg.pose.position.x=targetpose_msg.pose.position.x+x_dist;
    pose_msg.pose.position.y=targetpose_msg.pose.position.y+y_dist;
    pose_msg.pose.position.z=targetpose_msg.pose.position.z+z_dist;
    pose_msg.pose.orientation.x=q.x();
    pose_msg.pose.orientation.y=q.y();
    pose_msg.pose.orientation.z=q.z();
    pose_msg.pose.orientation.w=q.w();

    ROS_INFO("The generate pose x is: %f",pose_msg.pose.position.x);
    ROS_INFO("The generate pose y is: %f",pose_msg.pose.position.y);
    ROS_INFO("The generate pose z is: %f",pose_msg.pose.position.z);
    ROS_INFO("The generate qx is: %f",pose_msg.pose.orientation.x);
    ROS_INFO("The generate qy is: %f",pose_msg.pose.orientation.y);
    ROS_INFO("The generate qz is: %f",pose_msg.pose.orientation.z);
    ROS_INFO("The generate qw is: %f",pose_msg.pose.orientation.w);

    return pose_msg;
}

bool PickPlace::workspace(double x,double y,double z)
{
    if((abs(x+0.0955)<=x_workspace)&&(abs(y)<=y_workspace)&&((z+0.433)<=z_workspace)&&(z>0))
    {
        ROS_INFO("The target is in the workspace.");
        return true;
    }
    else
    {
        ROS_INFO("The target is not in the workspace.");
        ROS_INFO("Start to transform the replan mode");
        mode=3;
        return false;
    }
}

//通过action控制三指
bool PickPlace::gripper_three_action(double finger_turn)
{
    if(robot_connected_==false)
    {
        if(finger_turn>0.5*FINGER_MAX)
        {
            gripper_group_->setNamedTarget("Close");
        }
        else
        {
            gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }

    if(finger_turn<0)
    {
        finger_turn=0.0;
    }
    else
    {
        finger_turn=std::min(finger_turn,FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1=finger_turn;
    goal.fingers.finger2=goal.fingers.finger1;
    goal.fingers.finger3=goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if(finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action time-out");
        return false;
    }
}

bool PickPlace::gripper_two_action(double finger_turn)
{}


void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group)
{
    bool replan=true;
    int count=0;
    position_replan_=false;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while(replan==true&&ros::ok())
    {
        count=0;
        result_=false;

        double planning_time;
        while(result_==false && count<5)
        {
            count++;
            planning_time=20+count*10;
            ROS_INFO("Planning time is: %f sec",planning_time);
            group.setPlanningTime(planning_time);
            result_=(group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("At attemp: %d",count);
            ros::WallDuration(0.1).sleep();
            if(result_==false)
            {
                std::cout<<"Please input q to adjust car's angle,others to skip:";
                std::cin>>pause_;
                ros::WallDuration(0.5).sleep();
                if(pause_=="q" || pause_=="Q")
                {
                    mode=3;
                    position_replan_=true;
                    return;
                }
            } 
        }

        if(result_==true)
        {
            std::cout<<"Plan success at attemp: "<<count<<std::endl;

            replan=false;
            display_trajectory(my_plan);
            std::cout<<"Please input e to execute the plan,r to replan,others to skip:";
            std::cin>>pause_;
            ros::WallDuration(0.5).sleep();
            if(pause_=="r" || pause_=="R")
            {
                replan=true;
            }
            else
            {
                replan=false;
            }
        }
        else
        {
            std::cout<<"Exit since plan failed until reach maximun attemp"<<count<<std::endl;
            replan=false;
            break;
        }
    }

    if(result_==true)
    {
        if(pause_=="e" || pause_=="E")
        {
            group.execute(my_plan);
        }
    }
    ros::WallDuration(1.0).sleep();
}

bool PickPlace::define_pick(double x,double y,double z)
{
    clear_workscene();
    ros::WallDuration(1.0).sleep();
    build_workscene();
    ros::WallDuration(1.0).sleep();

    ROS_INFO("Please press any key to send robot to the Home");
    std::cin>>pause_;
    arm_group_->clearPathConstraints();
    arm_group_->setNamedTarget("Home");
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    add_target(0.5,0.04,x,y,z);
    //add_obstacle();
    ros::WallDuration(0.1).sleep();

    /*ROS_INFO_STREAM("Press any key to move robot to start pose");
    std::cin>>pause_;
    arm_group_->setPoseTarget(start_pose_);
    evaluate_plan(*arm_group_);*/

    ROS_INFO_STREAM("Planning to pre-grasp position");
    arm_group_->setPoseTarget(pregrasp_pose_);
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    ROS_INFO_STREAM("Approaching to grasp position");
    arm_group_->setPoseTarget(grasp_pose_);
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    ROS_INFO_STREAM("Grasping...");
    add_attached_obstacle();
    gripper_three_action(0.75*FINGER_MAX);

    ROS_INFO_STREAM("Planning to post-grasp position");
    arm_group_->setPoseTarget(postgrasp_pose_);
    evaluate_plan(*arm_group_);
    if(position_replan_) return false; 

    /*
    ROS_INFO_STREAM("Releasing gripper...");
    gripper_three_action(0.0);

    clear_workscene();
    ROS_INFO_STREAM("Press any key to quit the grasp");
    std::cin>>pause_;
    */

    return true;
}

bool PickPlace::define_place()
{
    ROS_INFO_STREAM("Press any key to preplace the place");
    std::cin>>pause_;
    arm_group_->setPoseTarget(preplace_pose_);
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    ROS_INFO_STREAM("Approaching to place position");
    arm_group_->setPoseTarget(place_pose_);
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    ROS_INFO_STREAM("Releasing gripper...");
    gripper_three_action(0.0);
    remove_attached_obstacle();

    ROS_INFO_STREAM("Planning to post_place position");
    arm_group_->setPoseTarget(postplace_pose_);
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    /*ROS_INFO_STREAM("Planning to start position");
    arm_group_->setPoseTarget(start_pose_);
    evaluate_plan(*arm_group_);*/

    ROS_INFO("Please press any key to send robot to the Home");
    std::cin>>pause_;
    arm_group_->clearPathConstraints();
    arm_group_->setNamedTarget("Home");
    evaluate_plan(*arm_group_);
    if(position_replan_) return false;

    ROS_INFO_STREAM("Press any key to quit...");
    std::cin>>pause_;
    clear_workscene();
    return true;
}


bool PickPlace::pick(double x,double y,double z)
{
    define_grasp_cartesian_pose(x,y,z);

    result_=false;
    if(define_pick(x,y,z)) return true;
    else return false;
}

bool PickPlace::place(double x,double y,double z)
{
    define_place_cartesian_pose(x,y,z);

    result_=false;
    if(define_place()) return true;
    else return false;
}
