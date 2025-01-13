//
// Created by felixd on 10/1/15.
//

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

// Defines DOF_JOINTS.
#include <allegro_hand_driver/AllegroHandDrv.h>
using namespace allegro;

#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "bhand/BHand.h"
#include <std_msgs/msg/float32.hpp>

// Forward declaration.
class AllegroHandDrv;

#define ALLEGRO_CONTROL_TIME_INTERVAL 0.002

// Topic names: current & desired JointState, named grasp to command.
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";

class AllegroNode: public rclcpp::Node {
 public:

  AllegroNode(const std::string nodeName, bool sim = false);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr desired);

  void ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg);


  virtual void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    RCLCPP_ERROR(rclcpp::get_logger("allegro_node"), "Called virtual function!");
  };

  rclcpp::TimerBase::SharedPtr startTimerCallback();

  void timerCallback();

 protected:

  double position_offset[DOF_JOINTS] = {0.0};

  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};

  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};

  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};

  double desired_torque[DOF_JOINTS] = {0.0};

  int status_interval = 0;

  // ROS stuff
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Time t_last_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
  // Store the current and desired joint states.
  sensor_msgs::msg::JointState current_joint_state;
  sensor_msgs::msg::JointState desired_joint_state;

  // ROS Time
  rclcpp::Time tstart;
  rclcpp::Time tnow;
  double dt;

  // CAN device
  allegro::AllegroHandDrv *canDevice;
  std::mutex *mutex;

  // Flags
  int lEmergencyStop = 0;
  long frame = 0;

  double motion_time = 0.6;//
  double force_get = 10.0f;//

  BHand *pBHand = NULL;
};

#endif //PROJECT_ALLEGRO_NODE_COMMON_H
