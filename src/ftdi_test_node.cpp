
#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/String.h"

#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <ftdi.h>

#include <string>



unsigned char buf[1];

std_msgs::Bool boolVal;
bool buffer = false;
int increment = 1;


int main(int argc, char **argv){

  // ROS
  ros::init(argc, argv, "ftdi_test_node");
  ros::NodeHandle nh("~");

  ros::Publisher pinPub0 = nh.advertise<std_msgs::Bool>("/pin0", 1000);
  ros::Publisher pinPub1 = nh.advertise<std_msgs::Bool>("/pin1", 1000);
  ros::Publisher pinPub2 = nh.advertise<std_msgs::Bool>("/pin2", 1000);
  ros::Publisher pinPub3 = nh.advertise<std_msgs::Bool>("/pin3", 1000);
  ros::Publisher pinPub4 = nh.advertise<std_msgs::Bool>("/pin4", 1000);
  ros::Publisher pinPub5 = nh.advertise<std_msgs::Bool>("/pin5", 1000);
  ros::Publisher pinPub6 = nh.advertise<std_msgs::Bool>("/pin6", 1000);
  ros::Publisher pinPub7 = nh.advertise<std_msgs::Bool>("/pin7", 1000);

  ros::Publisher pinPub[8];
  for(int i=0; i<8; i++){
    std::string topic = "/pin" + std::to_string(i);
    pinPub[i] = nh.advertise<std_msgs::Bool>(topic, 1000);
  }

  ros::Rate loop_rate(50);  //Specify loop rate/frequency in Hz
  // /ROS

  int count = 0;
  while (ros::ok())
  {

    buffer = !buffer;
    // printf("%d %d\n", (count/2)%8, buffer);
    boolVal.data = buffer;

    pinPub[(count/2)%8].publish(boolVal);
    if(buffer) loop_rate.sleep();

    // go the other way



    ros::spinOnce();

    count += increment;
    if((count/2) > 7 || count < 0){
      increment = increment * -1;
      count += increment;
    }


    loop_rate.sleep();
  }



  return 0;
}
