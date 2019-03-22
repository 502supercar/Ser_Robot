#ifndef MY_PICK_PLACE_H
#define MY_PICK_PLACE_H

#include <ros/ros.h>
#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

const int x_workspace=0.85;
const int y_workspace=0.85;
const int z_workspace=1.1;

namespace kinova
{
    class PickPlace
    {
        public:
            PickPlace(ros::NodeHandle &nh);
            ~PickPlace();
            void pick(double x,double y,double z);
            void place(double x,double y,double z);
            bool workspace(double x,double y,double z);
        private:
            ros::NodeHandle nh_;

            actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;

            moveit::planning_interface::MoveGroupInterface* arm_group_;
            moveit::planning_interface::MoveGroupInterface* gripper_group_;
            robot_model::RobotModelPtr robot_model_;
            robot_state::RobotStatePtr robot_state_;
            const robot_state::JointModelGroup* joint_model_group_;

            moveit_visual_tools::MoveItVisualTools* visual_tools_;
            planning_scene::PlanningScenePtr planning_scene_;
            planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

            moveit_msgs::CollisionObject co_;
            moveit_msgs::AttachedCollisionObject aco_;
            moveit_msgs::PlanningScene planning_scene_msg_;
            //moveit_msgs::DisplayTrajectory display_trajectory_;

            ros::Publisher pub_co_;
            ros::Publisher pub_aco_;
            ros::Publisher pub_planning_scene_diff_;
            //ros::Publisher pub_trajectory_;
            ros::Subscriber sub_pose_;
            ros::Subscriber sub_joint_;

            std::vector<std::string> joint_name_;
            std::vector<double> joint_value_;

            bool result_;

            std::string pause_;
            std::string robot_type_;
            bool robot_connected_;

            boost::mutex mutex_state_;
            boost::mutex mutex_pose_;
            sensor_msgs::JointState current_state_;
            geometry_msgs::PoseStamped current_pose_;

          /*  std::vector<double> start_joint_;
            std::vector<double> grasp_joint_;
            std::vector<double> pregrasp_joint_;
            std::vector<double> postgrasp_joint_;*/

           // geometry_msgs::PoseStamped start_pose_;
            geometry_msgs::PoseStamped grasp_pose_;
            geometry_msgs::PoseStamped pregrasp_pose_;
            geometry_msgs::PoseStamped postgrasp_pose_;
            geometry_msgs::PoseStamped place_pose_;
            geometry_msgs::PoseStamped preplace_pose_;
            geometry_msgs::PoseStamped postplace_pose_;

            void add_obstacle();
            void clear_obstacle();
            void add_attached_obstacle();
            void remove_attached_obstacle();
            void add_target(double radius,double height,double x,double y,double z);
            void build_workscene();
            void clear_workscene();

            void display_trajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan);

            void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
            void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);

            geometry_msgs::PoseStamped generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg,double dist,double azimuth,double polar,double rot_gripper_z);
            void define_grasp_cartesian_pose(double x,double y,double z);
            void define_place_cartesian_pose(double x,double y,double z);
            //void defien_joint_values();

            bool gripper_three_action(double finger_turn);
            bool gripper_two_action(double finger_turn);
            void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group);
            bool define_pick(double x,double y,double z);
            bool define_place();

    };
}

#endif