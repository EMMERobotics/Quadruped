#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "tester");
  
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("servo", 1000);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;

    if (count % 20 < 10) {
        ss << "#1P1800T1000\r\n"  << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
    }

    else {
        ss << "#1P1500T1000\r\n"  << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
    }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
