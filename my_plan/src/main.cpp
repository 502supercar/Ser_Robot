#include <my_pick_place.h>
#include <mobile_base.h>

extern int mode;

int main(int argc,char** argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Mobile_Base mobilebase(node);     //创建一个Mobile_Base对象
  kinova::PickPlace pick_place(node);

  std::string a;
  std::cout<<"Please enter any key to start!"<<std::endl;
  std::cin>>a;

  //replan_flag = false;

  mode=0;

  while(ros::ok())
  {
    //mode=0为小车模式，mode=1为机械臂pick模式,mode=2为机械臂place模式，mode=3为旋转调整位置模式
    switch(mode)
    {
      case 0:
        mobilebase.getGoalFromObject(1.6,0,0);
        if(mobilebase.moveToGoal())
        {
          mode=1;
        }
        else
        {
          std::cout<<"Failed moving goal."<<std::endl;
          return -1;
        }
        break;
      case 1:
        //workspace为false,则函数内部mode=2;
        if(pick_place.workspace(0.7,0.5,0.3))
        { 
          //pick和place函数当规划失败时，可选择将mode置为2;
          if(pick_place.pick(0.7,0.5,0.3)) mode=2;  
          //mode=3;
        }
        break;
      case 2:
        if(pick_place.workspace(0.4,-0.5,0.45))
        { 
          //pick和place函数当规划失败时，可选择将mode置为2;
          if(pick_place.place(0.4,-0.5,0.45)) mode=4;
          //mode=3;
        }
        break;
      case 3:
        if(mobilebase.carRotation(30))
        {
          mode=1;
        }
        break;
      default:
        std::cout<<"Press w to start mode 0,other to stop the program.";
        std::cin>>a;
        if(a=="w" || a=="W")
        {
          mode=0;
        }
        else
        {
          return 1;
        }
        break;
    }
  }

  ros::spin();
  return 0;
}
