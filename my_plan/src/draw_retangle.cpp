#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
//#include <kinova_driver/kinova_ros_types.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"draw_retangle");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //sleep(10);

  bool success;
  

  //声明move_group、planning_scene、rviz、jointmodelgroup、robotstate的控制接口
  const std::string PLANNING_GROUP("arm");
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  robot_state::RobotStatePtr robot_state_ptr=move_group.getCurrentState();
  const robot_state::JointModelGroup* joint_model_group=move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //delete all the markers,以防其他节点的markers干扰
  visual_tools.deleteAllMarkers();

  ROS_INFO("Reference frame: %s",move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s",move_group.getEndEffectorLink().c_str());

  //读取当前末端执行器的位姿
  geometry_msgs::PoseStamped current_pose=move_group.getCurrentPose();
  ROS_INFO("The current position x is %f",current_pose.pose.position.x);
  ROS_INFO("The current position y is %f",current_pose.pose.position.y);
  ROS_INFO("The current position z is %f",current_pose.pose.position.z);
  ROS_INFO("The current quaternion x is %f",current_pose.pose.orientation.x);
  ROS_INFO("The current quaternion y is %f",current_pose.pose.orientation.y);
  ROS_INFO("The current quaternion z is %f",current_pose.pose.orientation.z);
  ROS_INFO("The current quaternion w is %f",current_pose.pose.orientation.w);
/*
  geometry_msgs::Pose target_pose;
  target_pose.position.x=current_pose.pose.position.x+0.1;
  target_pose.position.y=current_pose.pose.position.y;
  target_pose.position.z=current_pose.pose.position.z;
  target_pose.orientation.x=current_pose.pose.orientation.x;
  target_pose.orientation.y=current_pose.pose.orientation.y;
  target_pose.orientation.z=current_pose.pose.orientation.z;
  target_pose.orientation.w=current_pose.pose.orientation.w;
  move_group.setPoseTarget(target_pose);

  //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
     ROS_INFO("Plan failed");
     return -1;
  }
  else
  {
    ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }

  target_pose.position.z+=0.2;
  move_group.setPoseTarget(target_pose);
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
     ROS_INFO("Plan failed");
     return -1;
  }
  else
  {
    ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }

  target_pose.position.y+=0.2;
  move_group.setPoseTarget(target_pose);
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
     ROS_INFO("Plan failed");
     return -1;
  }
  else
  {
    ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }

  target_pose.position.z-=0.2;
  move_group.setPoseTarget(target_pose);
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
     ROS_INFO("Plan failed");
     return -1;
  }
  else
  {
    ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }

  target_pose.position.y-=0.2;
  move_group.setPoseTarget(target_pose);
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
     ROS_INFO("Plan failed");
     return -1;
  }
  else
  {
    ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }
*/


  //读取当前关节信息
  std::vector<double> current_joint_values;
  current_joint_values=move_group.getCurrentJointValues();
  for(int i=0;i<6;i++)
  {
      ROS_INFO("Joint%d : %f",i+1,current_joint_values[i]);
  }
  
  /*
  //关节空间规划
  std::vector<double> target_joint(current_joint_values);
  target_joint[0]+=30;
  move_group.setJointValueTarget(target_joint);
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
    ROS_INFO("Plan failed!");
    return -1;
  }
  else
  {
     ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }
*/

  /*
  //加进约束
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name="j2n6s300_end_effector";
  ocm.header.frame_id="base_link";
  ocm.orientation.w=1.0;
  ocm.absolute_x_axis_tolerance=1.0;
  ocm.absolute_y_axis_tolerance=1.0;
  ocm.absolute_z_axis_tolerance=1.0;
  ocm.weight=1.0;

  moveit_msgs::Constraints my_constraints;
  my_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(my_constraints);


  geometry_msgs::Pose target_pose2;
  target_pose2.position.x=current_pose.pose.position.x+0.4;
  target_pose2.position.y=current_pose.pose.position.y;
  target_pose2.position.z=current_pose.pose.position.z+0.4;
  target_pose2.orientation.x=current_pose.pose.orientation.x;
  target_pose2.orientation.y=current_pose.pose.orientation.y;
  target_pose2.orientation.z=current_pose.pose.orientation.z;
  target_pose2.orientation.w=current_pose.pose.orientation.w;

  move_group.setPoseTarget(target_pose2);  

  move_group.setPlanningTime(25.0);
  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
    ROS_INFO("Plan failed!");
    return -1;
  }
  else
  {
     ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);

    move_group.clearPathConstraints();
  }
  */

  /*
  //cartesian paths
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose pose1(current_pose.pose);
  waypoints.push_back(pose1);

  pose1.position.x+=0.1;
  waypoints.push_back(pose1);

  pose1.position.z+=0.2;
  waypoints.push_back(pose1);
  pose1.position.y+=0.2;
  waypoints.push_back(pose1);
  pose1.position.z-=0.2;
  waypoints.push_back(pose1);
  pose1.position.y-=0.2;
  waypoints.push_back(pose1);

  moveit_msgs::RobotTrajectory trajectory1;
  double fraction=move_group.computeCartesianPath(waypoints,0.01,0.0,trajectory1);
  ROS_INFO("Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  visual_tools.publishTrajectoryLine(trajectory1,joint_model_group);
  visual_tools.trigger();

  ROS_INFO("Sleep 10S...");
  sleep(10);

  my_plan.trajectory_=trajectory1;
  move_group.execute(my_plan);
*/
/*
  //加进障碍物
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id="base_link";
  collision_object.id="box1";

  shape_msgs::SolidPrimitive primitive;
  primitive.type=primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0]=0.2;
  primitive.dimensions[1]=0.05;
  primitive.dimensions[2]=0.2;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w=1.0;
  box_pose.position.x=0.3;
  box_pose.position.y=-0.2;
  box_pose.position.z=0.9;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation=collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
*/
  /*
  geometry_msgs::Pose target_pose;
  target_pose.position.x=0.4;
  target_pose.position.y=-0.317;
  target_pose.position.z=0.6337;
  target_pose.orientation.x=0.229;
  target_pose.orientation.y=0.6823;
  target_pose.orientation.z=0.6883;
  target_pose.orientation.w=0.09;
  move_group.setPoseTarget(target_pose);

  success=(move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
  {
    ROS_INFO("Plan failed!");
    return -1;
  }
  else
  {
     ROS_INFO("Plan succeed!");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools.trigger();
    //sleep(5);
    move_group.execute(my_plan);
  }
  */

  ros::Publisher vis_pub=node_handle.advertise<visualization_msgs::Marker>("visualization_marker",1);

  visualization_msgs::Marker marker;
  marker.header.frame_id="base_link";
  //marker.header.stamp=ros::Time();
  marker.ns="my_namespace";
  marker.id=0;
  marker.type=visualization_msgs::Marker::SPHERE;
  marker.action=visualization_msgs::Marker::ADD;

  marker.pose.position.x=10.0;
  marker.pose.position.y=10.0;
  marker.pose.position.z=1.0;
  marker.pose.orientation.w=1.0;
  marker.scale.x=1.0;
  marker.scale.y=0.1;
  marker.scale.z=0.1;
  marker.color.a=1.0;
  marker.color.r=0.0;
  marker.color.g=1.0;
  marker.color.b=0.0;

  marker.mesh_resource="package:://kinova_description/meshes/door.stl";

  ros::Rate rate(10);
  while(node_handle.ok())
  {
    vis_pub.publish(marker);
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
