#ifndef MUJOCO_INTERFACE_H
#define MUJOCO_INTERFACE_H

#include "control_base.h"
#include "math_type_define.h"
#include <mujoco_ros_msgs/JointSet.h>
#include <mujoco_ros_msgs/JointState.h>
#include <mujoco_ros_msgs/SensorState.h>
#include <mujoco_ros_msgs/JointInit.h>


namespace dyros_red_controller
{

class MujocoInterface : public ControlBase{
public:
  MujocoInterface(ros::NodeHandle &nh, double Hz); // constructor for initialize node
  virtual ~MujocoInterface() { mujocoStop(); }

  virtual void update() override; // update controller based on readdevice
  virtual void compute() override; // compute algorithm and update all class object
  virtual void writeDevice() override; // publish to actuate devices
  virtual void wait() override;

private:  // CALLBACK
  void simulationConnectorCallback(const std_msgs::Float32ConstPtr& msg);

  void jointCallback(const mujoco_ros_msgs::JointStateConstPtr& msg);
  void sensorCallback(const mujoco_ros_msgs::SensorStateConstPtr& msg);



private:

  ros::Publisher mujoco_joint_set_pub_;
  ros::Publisher mujoco_sim_command;
  ros::Subscriber mujoco_sim_status_sub_;
  ros::Subscriber mujoco_sensor_status_sub_;
  ros::Subscriber mujoco_joint_status_sub_;

  mujoco_ros_msgs::JointSet joint_set_msg_;

  float simulation_time_; // from v-rep simulation time
  void mujocoStop();

  ros::Rate rate_;

};

}


#endif // MUJOCO_INTERFACE_H
