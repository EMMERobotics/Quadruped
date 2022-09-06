#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <quadruped_main/leg_comm_msg.h>
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

void parse_motor_command(ros::Publisher pub, Leg leg) {

  quadruped_main::leg_comm_msg msg;
  
  // geometry_msgs::Twist msg1;

  msg.leg         = leg.leg_i;
  msg.hipAngle    = leg.hipAngle;
  msg.femurAngle  = leg.femurAngle;
  msg.tibiaAngle  = leg.tibiaAngle;

  pub.publish(msg);
  //ROS_INFO("%f", motor_command.x);
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_node");
  ros::NodeHandle n;
  ros::Publisher motor_comm_pub = n.advertise<quadruped_main::leg_comm_msg>("motor_command", 1000);
  // ros::Publisher motor_comm_pub = n.advertise<geometry_msgs::Twist>("motor_command", 1000);
  ros::Rate loop_rate(100);

  int count = 0;

  while (ros::ok())
  {

    geometry_msgs::Twist motor_command;

    //GET_COMMAND()

    gait_controller(robot_state);
    
    parse_motor_command(motor_comm_pub, leg_FL);
    parse_motor_command(motor_comm_pub, leg_FR);
    parse_motor_command(motor_comm_pub, leg_BL);
    parse_motor_command(motor_comm_pub, leg_BR);

    //ROS_INFO("%f", motor_command.x);

    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }

  return 0;
}