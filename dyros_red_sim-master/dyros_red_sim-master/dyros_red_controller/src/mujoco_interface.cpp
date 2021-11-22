#include "dyros_red_controller/mujoco_interface.h"



namespace dyros_red_controller {

MujocoInterface::MujocoInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz)
{




    mujoco_joint_set_pub_=nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set",1);

    //mujoco_sim_command=nh.advertise<mujoco_ros_msgs::

    //mujoco_sim_status_sub_=nh.subscribe("/mujoco_ros_interface/sim_status", 100, &MujocoInterface::simStatusCallback,this);
    mujoco_sensor_status_sub_=nh.subscribe("/mujoco_ros_interface/sensor_states",1,&MujocoInterface::sensorCallback,this,ros::TransportHints().tcpNoDelay(true));
    mujoco_joint_status_sub_=nh.subscribe("/mujoco_ros_interface/joint_states",1,&MujocoInterface::jointCallback,this,ros::TransportHints().tcpNoDelay(true));


}

void MujocoInterface::update()
{
  ControlBase::update();

}
void MujocoInterface::compute()
{
  ControlBase::compute();
}

void MujocoInterface::writeDevice()
{


  for(int i=0;i<total_dof_;i++) {
    //std::cout<<"for loop"<<std::endl;
    //std::cout<<"done : "<<i<<" ::: effort : " <<torque_desired(i)<<std::endl;
    joint_set_msg_.position[i] = position_desired(i);
    joint_set_msg_.effort[i] = torque_desired(i);


  }
  vrep_joint_set_pub_.publish(joint_set_msg_);
  vrepStepTrigger();
}

}
