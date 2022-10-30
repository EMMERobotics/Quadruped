#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <quadruped_main/leg_comm_msg.h>
#include "kinematics/kinematics.h"
#include <quadruped_main/con_msg.h>

#define DEBUG
//#define VERBOSE


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
    .pairs = true,

    .com_vx = 0,
    .com_vy = 0,
    .com_vz = 0,
    .com_roll = 0,

    .exphase = STILL,
    .comphase = STILL,

    .mode = CRAWL,
    .crawlphase = BACK_RIGHT,
    .crawl_completed = false

};


void parse_motor_command(ros::Publisher pub, Leg leg) {

  quadruped_main::leg_comm_msg msg;

  msg.leg         = leg.leg_i;
  msg.hipAngle    = leg.hipAngle;
  msg.femurAngle  = leg.femurAngle;
  msg.tibiaAngle  = leg.tibiaAngle;

  pub.publish(msg);
  //ROS_INFO("%f", motor_command.x);
  
}

void joy_callback(const quadruped_main::con_msg::ConstPtr& msg) {

  #ifdef VERBOSE
    ROS_INFO("Getting Controller");
  #endif

  #ifdef DEBUG
    //ROS_INFO("Value x1 is: [%i]", msg->val_x1);
    //ROS_INFO("Value x1 is: [%i]", msg->val_y1);
    //ROS_INFO("Value x2 is: [%i]", msg->val_x2);
    //ROS_INFO("Value y2 is: [%i]", msg->val_y2);
  #endif

  robot_state.com_vx = msg->val_y1;
  robot_state.com_vy = msg->val_x1;
  robot_state.com_vz = msg->val_y2;
  robot_state.com_roll = msg->val_x2;

  if (msg->b_x == 1) robot_state.mode = NORM;
  if (msg->b_o == 1) robot_state.mode = CRAWL;
  if (msg->b_sq == 1) robot_state.mode = RPY;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_node");
  ros::NodeHandle n;
  ros::Publisher motor_comm_pub = n.advertise<quadruped_main::leg_comm_msg>("motor_command", 1000);
  ros::Subscriber sub = n.subscribe("joy_val", 10000, joy_callback);
  ros::Rate loop_rate(100);

  #ifdef VERBOSE
    ROS_INFO("Kinemtaics Node Initilized");
  #endif
  robot_state.exphase = STILL;
  while (ros::ok())
  {
    
    gait_controller(robot_state);

    #ifdef VERBOSE
      ROS_INFO("Publishing Motor");
    #endif

    parse_motor_command(motor_comm_pub, leg_FL);
    parse_motor_command(motor_comm_pub, leg_FR);
    parse_motor_command(motor_comm_pub, leg_BL);
    parse_motor_command(motor_comm_pub, leg_BR);

    #ifdef DEBUG
    ROS_INFO("com: %d", robot_state.comphase);
    //ROS_INFO("ph: %d", robot_state.mode);
    ROS_INFO("ex:    %d", robot_state.exphase);
    ROS_INFO("tic:    %d", robot_state.ticks);
    //ROS_INFO("HIP:       %f", leg_FL.hipAngle);
    //ROS_INFO("FEM:         %f", leg_FL.femurAngle);
    //ROS_INFO("TIB:            %f", leg_FL.tibiaAngle);
    ROS_INFO("CRAWL:    %d", robot_state.crawlphase);
    //ROS_INFO("pairs:           %d", robot_state.pairs);
    #endif
    ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}
