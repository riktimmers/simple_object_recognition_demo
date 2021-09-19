#include <ros/ros.h>
#include "perception/perception.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_node"); 
  ros::NodeHandle node_handle; 

  Perception perception(node_handle);
  
  ros::spin();
}