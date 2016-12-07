#include "PidController/pid_controller_node.h"

/// Main function. ROS node is initialized here and main loop executed.
/// \param argc The number of command line arguments.
/// \param argv Command line arguments
/// \return Returns 0 if the ROS node is properly terminated.
///
int main(int argc, char **argv)
{
  // Set up ROS node.
  ros::init(argc, argv, "pid_controller_node");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  // ROS node rate (also the PID algorithm rate)
  int rate;
  // PID proportional gain
  double kp;
  // PID integral gain
  double ki;
  // PID derivative gain
  double kd;
  // Maximal and minimal control value
  double u_max, u_min;
  // message used for publishing actuator control value
  std_msgs::Float32 actuator_msg;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, 100);
  private_node_handle_.param("kp", kp, 0.0);
  private_node_handle_.param("ki", ki, 0.0);
  private_node_handle_.param("kd", kd, 0.0);
  private_node_handle_.param("u_max", u_max, std::numeric_limits<double>::infinity());
  private_node_handle_.param("u_min", u_min, -std::numeric_limits<double>::infinity());


  // Create a new NodeExample object.
  PidControllerRos *pid_controller = new PidControllerRos(kp, ki, kd);
  pid_controller->setUMax(u_max);
  pid_controller->setUMin(u_min);

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber ref_sub = n.subscribe("reference", 1, &PidControllerRos::referenceCallback, pid_controller);
  ros::Subscriber meas_sub = n.subscribe("measurement", 1, &PidControllerRos::measurementCallback, pid_controller);

  // Create a publisher to publish a control value for the actuator
  ros::Publisher actuator_pub = n.advertise<std_msgs::Float32>("actuator", 1);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    ros::spinOnce();
    actuator_msg.data = pid_controller->compute(pid_controller->getReference(), pid_controller->getMeasurement());
    actuator_pub.publish(actuator_msg);
    r.sleep();
  }

  return 0;
} // end main()
