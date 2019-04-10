#include <my_pick_place.h>
#include <mobile_base.h>


extern int mode;
extern double position[3];
double buchang[3];
const double r=0.03;

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


 // mobilebase.initRotation();

  mode=0;
  srand((unsigned int)time(NULL));

  //pick_place.home();
  
  while(ros::ok())
  {
    //mode=0为小车模式，mode=1为机械臂pick模式,mode=2为机械臂place模式，mode=3为旋转调整位置模式
    switch(mode)
    {
      case 0:
      ROS_INFO("Into the mode 0");
      mobile_goal object;
      object = mobilebase.findObject();
      ROS_INFO("object: x= %f, y= %f",object.x,object.y);
      if(mobilebase.getGoalFromObject(object.x,object.y,0))
      {
          mobilebase.moveToGoal("base_link");
      }
      ROS_INFO("Mission complete");
	  mode =1;
      ros::WallDuration(2).sleep();
      pick_place.home();
        break;
      case 1:
        //workspace为false,则函数内部mode=2;
        ROS_INFO("Into the mode 1");
        //ros::WallDuration(2).sleep();
        if(pick_place.tf_listen()==false)
        {
          mode=3;
          continue;
        }
        ROS_INFO("The position x is:%f",position[0]);
        ROS_INFO("The position y is:%f",position[1]);
        ROS_INFO("The position z is:%f",position[2]);
        buchang[0]=r*position[0]/sqrt(pow(position[0],2)+pow(position[1],2))+0.041;
        buchang[1]=r*position[1]/sqrt(pow(position[0],2)+pow(position[1],2))-0.015;
        buchang[2]=0.012;
        ROS_INFO("The buchang0 is:%f",buchang[0]);
        ROS_INFO("The buchang1 is:%f",buchang[1]);
        if(pick_place.workspace(position[0]+buchang[0],position[1]+buchang[1],position[2]+buchang[2]))
        {
            //pick_place.home();
            if(pick_place.pick(position[0]+buchang[0],position[1]+buchang[1],position[2]+buchang[2])) mode=6;
            else mode=5;
        }
        break;
      case 2:
        if(pick_place.workspace(0.4,-0.5,0.45))
        { 
          //pick和place函数当规划失败时，可选择将mode置为2;
          if(pick_place.place(0.4,-0.5,0.45)) mode=7;
          //mode=3;
        }
        break;
      case 3: //捕获不到物体，旋转
        /*if(mobilebase.carRotation(30))
        {
          mode=1;
        }*/
        ROS_INFO("Into the mode 3.");
        mode=1;
        break;
      case 4:  //抓取点不在工作范围内
        ROS_INFO("Into the mode 4");
        mode=1;
        break;
      case 5: //机械臂规划不成功，重新规划
        ROS_INFO("Into the mode 5");
        if(pick_place.random_position(position[0]+buchang[0],position[1]+buchang[1],position[2]+buchang[2])) mode=1;
        break;
      case 6:
        ROS_INFO("Into the mode 6");
        mobile_goal target;
        target.x=0;
        target.y=0;
        target.theta =0;

        mobilebase.setTargetPose(target);
		mobilebase.moveToGoal("map");
        mode=7;
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

