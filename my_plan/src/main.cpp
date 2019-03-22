#include <my_pick_place.h>
#include <mobile_base.h>
/*
double grasp_x=0.7;
double grasp_y=0.5;
double grasp_z=0.3;

int main(int argc,char** argv)
{ 
    ros::init(argc, argv, "main");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
/*
    Mobile_Base mobilebase;     //创建一个Mobile_Base对象

    mobilebase.initRotation(node);        //初始化旋转定位

    mobilebase.moveToGoal(0.5, 0.0, 0.0, node); */     //发送目标点

    /*kinova::PickPlace pick_place(node);

    if(pick_place.workspace(grasp_x,grasp_y,grasp_z))
    {
      pick_place.pick(grasp_x,grasp_y,grasp_z);
    }
    else
    { 
      return -1;
    }


    if(pick_place.workspace(0.4,-0.5,0.45))
    {
      pick_place.place(0.4,-0.5,0.45);
    }
    else
    { 
      return -1;
    }

    ros::spin();
    return 0;
}*/

bool replan_flag;

int main(int argc,char** argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  char a;
  std::cout<<"Please enter any key to start!"<<std::endl;
  std::cin>>a;

  Mobile_Base mobilebase;     //创建一个Mobile_Base对象
  kinova::PickPlace pick_place(node);

  replan_flag = false;

  while(ros::ok())
  {
    if(replan_flag == false)
    {
        mobilebase.getGoalFromObject(1.6,0,0);
        mobilebase.moveToGoal(node);
    }
    else
    {
      replan_flag = false;
    }
    if(pick_place.workspace(0.7,0.5,0.3))
    {
      pick_place.pick(0.7,0.5,0.3);
    }
    else 
    {
      replan_flag=true;
      return -1;
    }
    if(pick_place.workspace(0.4,-0.5,0.45))
    {
      pick_place.place(0.4,-0.5,0.45);
    }
    else
    { 
      replan_flag=true;
      return -1;
    }
  }
  ros::spin();
  return 0;
}

