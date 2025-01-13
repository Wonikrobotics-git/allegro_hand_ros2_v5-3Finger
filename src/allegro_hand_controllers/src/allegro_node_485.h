#ifndef PROJECT_ALLEGRO_NODE_485_HPP
#define PROJECT_ALLEGRO_NODE_485_HPP


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "allegro_hand_driver/AllegroHandRS485Drv.h"
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>


// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp through RS-485 communication(e.g., three-finger pick, home position, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position.
//
// Author: Soohoon Yang(Hibo)
//
class AllegroNode485 : public rclcpp::Node {

 public:

    AllegroNode485();

    ~AllegroNode485();

    void initController(const std::string &whichPort);

    void libCmdCallback(const std_msgs::msg::String::SharedPtr msg);

    void doIt(bool polling);

protected:

   // Handles defined grasp commands (std_msgs/String).
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lib_cmd_sub;

   int fd = 0;

   double desired_position[9] = {0.0};

   int DOF_JOINTS = 9;

   std::string whichPort;  // Define RS-485 Portname.
};


#endif //PROJECT_ALLEGRO_NODE_485_HPP
