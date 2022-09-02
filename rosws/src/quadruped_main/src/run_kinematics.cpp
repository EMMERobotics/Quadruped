#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "kinematics/kinematics.h"



STATE robot_state = { 

    .dt = 10,
    .current_time = 0,

    //robot frame current pose   
    .c_x = 0,
    .c_y = 0,
    .c_z = 0,
    .c_R = 0,
    .c_P = 0,
    .c_Y = 0,
    
    .p_x = 0,
    .p_y = 0,
    .p_z = 0,

    .ticks = 0,
    .mode = 0,
    .pairs = true

};

 COMMAND command {
    
    .v_x = 0,
    .v_y = 0,
    .v_z = 0,
    .roll = 0,
    .pitch = 0,
    .yaw = 0

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_node");
  ros::NodeHandle n;
  ros::Publisher motor_comm_pub = n.advertise<geometry_msgs::Twist>("motor_command", 1000);
  ros::Rate loop_rate(100);

  int count = 0;

  while (ros::ok())
  {

    geometry_msgs::Twist motor_command;

    //GET_COMMAND()

    gait_controller(robot_state);

    //ROS_INFO("%f", motor_command.x);
    motor_comm_pub.publish(motor_command);

    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }

  return 0;
}