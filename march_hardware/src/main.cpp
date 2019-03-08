// Copyright 2018 Project March.
#include <cmath>

#include <unistd.h>

#include <ros/ros.h>

#include <bitset>

#include <march_hardware/Joint.h>
#include <march_hardware/March4.h>
#include "sensor_msgs/JointState.h"
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EtherCAT/EthercatSDO.h>

int main(int argc, char** argv)
{
  march4cpp::MARCH4 march4 = march4cpp::MARCH4();
  march4.startEtherCAT();

  if (!march4.isEthercatOperational())
  {
    ROS_FATAL("EtherCAT is not operational");
    return 0;
  }

  ros::init(argc, argv, "dummy");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  // Uncomment to allow actuation.
  march4.getJoint("test_joint").getIMotionCube().goToOperationEnabled();
  ROS_INFO("march4 initialized");

  ROS_INFO_STREAM("Angle: " << march4.getJoint("test_joint").getAngleRad());
  march4.getJoint("test_joint").getIMotionCube().actuateRadFixedSpeed(1.1, 0.261);
  march4.getJoint("test_joint").getIMotionCube().actuateRadFixedSpeed(0.8, 0.261);
  march4.getJoint("test_joint").getIMotionCube().actuateRadFixedSpeed(1.2, 0.261);

  // Publish and print joint position
  //    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("march/joint_states", 5);
  //    angleVal = march4.getJoint("test_joint").getAngleRad();
  //    printf("imc get: %f\n", angleVal);
  //    sensor_msgs::JointState joint_state;
  //    joint_state.header.stamp = ros::Time::now();
  //    joint_state.name = {"test_joint"};
  //    joint_state.position = {angleVal};
  //    pub.publish(joint_state);

  // Print final status
  sleep(1);
  march4.getJoint("test_joint")
      .getIMotionCube()
      .parseStatusWord(march4.getJoint("test_joint").getIMotionCube().getStatusWord());
  march4.getJoint("test_joint")
      .getIMotionCube()
      .parseMotionError(march4.getJoint("test_joint").getIMotionCube().getMotionError());
  march4.getJoint("test_joint")
      .getIMotionCube()
      .parseDetailedError(march4.getJoint("test_joint").getIMotionCube().getDetailedError());

  march4.stopEtherCAT();
}