#include <ros/ros.h>
#include <tf/transform_listener.h>

double position[3];
double tf_x[20];
double tf_y[20];
double tf_z[20];
    int data_num=0;
    //double position[3];
    double total_x=0.0;
    double total_y=0.0;
    double total_z=0.0;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"tf_listen_example");
  ros::NodeHandle n;

   tf::TransformListener listener;
  ros::Rate rate(10.0);
  /*while(n.ok())
  {
    tf::StampedTransform transform;
    try{
       // listener.waitForTransform("/zed_Link", "/object",ros::Time::now(),ros::Duration(3.0));
        listener.lookupTransform("/zed_left_camera_frame", "/object",ros::Time(0), transform);
        ROS_INFO("posititon x is:%f",transform.getOrigin().x());
        ROS_INFO("posititon y is:%f",transform.getOrigin().y());
        ROS_INFO("posititon z is:%f",transform.getOrigin().z());
       }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ROS_INFO("Can't receive the tf.");
        ros::Duration(1.0).sleep();
       }
    
    rate.sleep();
  }*/

    for(int i=0;i<20;)
    {
        tf::StampedTransform transform;
        try
        {
          listener.lookupTransform("/zed_left_camera_frame", "/object",ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ROS_INFO("Can't receive the tf.");
        ros::Duration(1.0).sleep();
       }
        tf_x[i]=transform.getOrigin().x();
        tf_y[i]=transform.getOrigin().y();
        tf_z[i]=transform.getOrigin().z();
        if(i>0)
        {
          if(fabs(tf_z[i]-tf_z[i-1])>0.001)  i++;
        }
        else
        {
          i++;
        }  
        ROS_INFO("i is:%d",i);
        ROS_INFO("x is:%f",tf_x[i]);
        ROS_INFO("y is:%f",tf_y[i]); 
        ROS_INFO("z is:%f",tf_z[i]);
        rate.sleep(); 
    }

    for(int i=0;i<20;i++)
    {
        int m=0;
        for(int j=0;j<20;j++)
        {
            if(abs(tf_z[i]-tf_z[j])<0.05) m++;
        }
        if(m>=15)
        {
            total_x+=tf_x[i];
            total_y+=tf_y[i];
            total_z+=tf_z[i];
            data_num++;
        }
    }

    position[0]=total_x/data_num;
    position[1]=total_y/data_num;
    position[2]=total_z/data_num;


  return 0;
}
