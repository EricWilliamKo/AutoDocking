#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <autodocking/irResult.h>


ros::Publisher cmd_vel_pub;
ros::Subscriber irValue_sub;

bool DockingStationFound = false;
int SignalLostCounter = 0;

autodocking::irResult result;

char robotMotion = 'G';
/*
Mode A Slow turn Right
Mode B Slow turn Left
Mode C Slow go foward
Mode D Fast turn Right
Mode E Fast turn Left
Mode F Fast go foward
*/
void irValueCallback(const autodocking::irResult &queue)
{
  if(queue.farLeft)
  result.farLeft = true;
  if(queue.farRight)
  result.farRight = true;
  if(queue.nearLeft)
  result.nearLeft = true;
  if(queue.nearRight)
  result.nearRight = true;
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
    
    if(result.nearRight || result.nearLeft){
    SignalLostCounter = 0;
    DockingStationFound = true;
    if(result.nearRight)
      robotMotion = 'A';
    if(result.nearLeft)
      robotMotion = 'B';
    if(result.nearLeft && result.nearRight)
      robotMotion = 'C';
    if(result.nearLeft && result.farRight)
      robotMotion = 'G';
  }else if(result.farRight || result.farLeft){
    SignalLostCounter = 0;
    DockingStationFound = true;
    if(result.farRight)
      robotMotion = 'D';
    if(result.farLeft)
      robotMotion = 'E';
    if(result.farRight && result.farLeft)
      robotMotion = 'F';
  }else{
      SignalLostCounter++;
    }
    
    //report robot motion to the cml window
    printf("Mode:%c",robotMotion);
    
    //Looking for docking station if docking station is not found
    if (!DockingStationFound) {
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      cmd  ->angular.z = 0.2; // const rotate until 
      cmd->linear.x = 0.0;
      cmd_vel_pub.publish(cmd);
    }
    
    if (DockingStationFound){
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      switch(robotMotion){
        case 'A':
        cmd->angular.z = -0.1;
        cmd->linear.x = -0.06;
        break;
        case 'B':
        cmd->angular.z = 0.1;
        cmd->linear.x = -0.06;
        break;
        case 'C':
        cmd->angular.z = -0.04;
        cmd->linear.x = -0.06;
        break;
        case 'D':
        cmd->angular.z = -0.2;
        cmd->linear.x = -0.1;
        break;
        case 'E':
        cmd->angular.z = 0.2;
        cmd->linear.x = -0.1;
        break;
        case 'F':
        cmd->angular.z = 0.07;
        cmd->linear.x = -0.1;
        break;
        case 'G':
        cmd->angular.z = 0;
        cmd->linear.x = -0.05;
        break;
        //default:
        //cmd->angular.z = 0;
        //cmd->linear.x = 0;
        //break;
      }
      cmd_vel_pub.publish(cmd);
    }
    
    if (SignalLostCounter>40){
	    DockingStationFound = false;
	    SignalLostCounter = 0;
	  }
    
    result.farLeft = false;
    result.farRight = false;
    result.nearLeft = false;
    result.nearRight = false;
    
    r.sleep();
  }
  return 0;
}
