#include "PidController/pid_controller_ros.h"

PidControllerRos::PidControllerRos()
{

}

PidControllerRos::PidControllerRos(double kp, double ki, double kd) :
  PidControllerBase::PidControllerBase(kp, ki, kd)
{

}

PidControllerRos::~PidControllerRos()
{

}

double PidControllerRos::getReference(void)
{
  return reference_;
}

double PidControllerRos::getMeasurement(void)
{
  return measurement_;
}

void PidControllerRos::referenceCallback(const std_msgs::Float32::ConstPtr &msg)
{
  reference_ = msg->data;
  //ROS_INFO("New reference received: %.2f ", reference_);
}

void PidControllerRos::measurementCallback(const std_msgs::Float32::ConstPtr &msg)
{
  measurement_ = msg->data;
  //ROS_INFO("New measurement received: %.2f ", measurement_);
}
