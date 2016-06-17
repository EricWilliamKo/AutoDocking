#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <autodocking/irResult.h>


ros::Publisher cmd_vel_pub;
ros::Subscriber irValue_sub;

bool DockingStationFound = false;
int SignalLostCounter = 0;

char robotMotion = 'G';
/*
Mode A Fast turn Right
Mode B Fast turn Left
Mode C Fast go foward
Mode D Slow turn Right
Mode E Slow turn Left
Mode F Slow go foward
*/
void irValueCallback(const autodocking::irResult &result)
{
  if(result.nearRight || result.nearLeft){
    SignalLostCounter = 0;
    DockingStationFound = true;
    if(result.nearRight)
      robotMotion = 'A';
    if(result.nearLeft)
      robotMotion = 'B';
    if(result.nearLeft && result.nearRight)
      robotMotion = 'C';
  }else if(result.farRight || result.farLeft){
    SignalLostCounter = 0;
    DockingStationFound = true;
    if(result.farRight)
      robotMotion = 'D';
    if(result.farLeft)
      robotMotion = 'E';
    if(result.farRight || result.farLeft)
      robotMotion = 'F';
  }else{
      SignalLostCounter++;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "IR_autoDocking");

  ros::NodeHandle nh;
  
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  irValue_sub = nh.subscribe("/irCode", 10, irValueCallback);

  ros::Rate r(50);

  while (ros::ok())
  {
    ros::spinOnce(); 
    
    //Looking for docking station if docking station is not found
    if (!DockingStationFound) {
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      cmd->angular.z = 0.2; // const rotate until 
      cmd->linear.x = 0.0;
      cmd_vel_pub.publish(cmd);
    }
    
    if (DockingStationFound){
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      switch(robotMotion){
        case 'A':
        cmd->angular.z = -0.1;
        cmd->linear.x = -0.1;
        case 'B':
        cmd->angular.z = 0.1;
        cmd->linear.x = -0.1;
        case 'C':
        cmd->angular.z = 0;
        cmd->linear.x = -0.1;
        case 'D':
        cmd->angular.z = -0.06;
        cmd->linear.x = -0.06;
        case 'E':
        cmd->angular.z = 0.06;
        cmd->linear.x = -0.06;
        case 'F':
        cmd->angular.z = 0;
        cmd->linear.x = -0.06;
        default:
        cmd->angular.z = 0;
        cmd->linear.x = 0;
      }
      cmd_vel_pub.publish(cmd);
    }
    
    if (SignalLostCounter>70){
	    DockingStationFound = false;
	    SignalLostCounter = 0;
	  }
    r.sleep();
  }
  return 0;
}
