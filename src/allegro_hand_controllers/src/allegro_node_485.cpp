#include "allegro_node_485.h"

// Define a map from string (received message) to internal Grasp algorithm.
std::map<std::string, std::vector<uint8_t>> bhand_grasps = {
    {"home",    {CMD_PLACE, CMD_PLACE + sizeof(CMD_PLACE)}},   
    {"grasp_3", {CMD_PICK_3F, CMD_PICK_3F + sizeof(CMD_PICK_3F)}},
    {"gravcomp", {CMD_GRAVITY, CMD_GRAVITY + sizeof(CMD_GRAVITY)}},  // gravity compensation
    {"off",      {CMD_TORQUE_OFF, CMD_TORQUE_OFF + sizeof(CMD_TORQUE_OFF)}},  // turn joints off
    {"PosRead1",{CMD_POS_READ_1, CMD_POS_READ_1 + sizeof(CMD_POS_READ_1)}},
    {"PosRead2",{CMD_POS_READ_2, CMD_POS_READ_2 + sizeof(CMD_POS_READ_2)}},
    {"PosRead3",{CMD_POS_READ_3, CMD_POS_READ_3 + sizeof(CMD_POS_READ_3)}}
};


AllegroNode485::AllegroNode485()
: Node("allegro_node_485")
{

  declare_parameter("port_info/which_port", "/dev/ttyUSB0");
  whichPort = get_parameter("port_info/which_port").as_string();

  initController(whichPort);

  lib_cmd_sub = this->create_subscription<std_msgs::msg::String>(
          "allegroHand/lib_cmd", 1, std::bind(&AllegroNode485::libCmdCallback, this, std::placeholders::_1));
}

AllegroNode485::~AllegroNode485() {

}

// void AllegroNode485::libCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [%s]", msg->data.c_str());
//   const std::string lib_cmd = msg->data.c_str();

//   // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
//   // normally (case-by-case basis), note these should *not* be in the map.
//  auto itr = bhand_grasps.find(msg->data);
//   if (itr != bhand_grasps.end()) {
//     std::string motion_type_str;
//     for (const auto &val : itr->second) {
//       motion_type_str += std::to_string(val) + " ";
//     }
//     RCLCPP_INFO(this->get_logger(), "motion type = %s", motion_type_str.c_str());

//     const std::string command = msg->data;

//     // 명령어가 PosRead로 시작하는지 확인
//     if (command.find("PosRead") == 0) {
//         auto itr = bhand_grasps.find(command);
//         if (itr != bhand_grasps.end()) {
//             // 명령어 전송
//             const auto& cmd = itr->second;
//             sendData(fd, cmd.data(), cmd.size(), true);  // false로 보내기
//     }
//     } else {
//         auto itr = bhand_grasps.find(command);
//         if (itr != bhand_grasps.end()) {
//             // 명령어 전송
//             const auto& cmd = itr->second;
//             sendData(fd, cmd.data(), cmd.size(), false);  // false로 보내기
//         } else {
//             RCLCPP_WARN(this->get_logger(),"Unknown command: [%s]", command.c_str());
//         }
//     }

//  } else if (lib_cmd.find("pdControl") == 0) {

//   // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
//   // normally (case-by-case basis), note these should *not* be in the map.

//     RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [pdControl]");
//     std::string num_str = lib_cmd.substr(9);

//     int pose_num = std::stoi(num_str);
//     RCLCPP_INFO(this->get_logger(), "PDControl Mode with pose number %d", pose_num);

//     std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
//     std::string file_path = pkg_path + "/pose/pose" + std::to_string(pose_num) + ".yaml";

//     std::ifstream infile(file_path);
//     if (!infile.good()) {
//       RCLCPP_WARN(this->get_logger(), "Pose file does not exist. Please select a different command.");
//       return;
//     }

//     YAML::Node node = YAML::LoadFile(file_path);
//     std::vector<double> positions = node["position"].as<std::vector<double>>();

//     for (int i = 0; i < DOF_JOINTS; i++) {
//       desired_position[i] = positions[i];
//     }

//     //command_place(_can_handle);

//   } else if (lib_cmd.find("moveit") == 0) {

//   // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
//   // normally (case-by-case basis), note these should *not* be in the map.

//     std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
//     std::string file_path = pkg_path + "/pose/pose_moveit.yaml";

//     std::ifstream infile(file_path);
//     if (!infile.good()) {
//       RCLCPP_WARN(this->get_logger(), "Pose file does not exist. Please select a different command.");
//       return;
//     }

//     YAML::Node node = YAML::LoadFile(file_path);
//     std::vector<double> positions = node["position"].as<std::vector<double>>();

//     for (int i = 0; i < DOF_JOINTS; i++) {
//       desired_position[i] = positions[i];
//     }
    
//     //command_place(_can_handle);
//   }
//   else {
//     std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
//         std::string file_path = pkg_path + "/pose/" + lib_cmd + ".yaml";

//     std::ifstream infile(file_path);
//     if (!infile.good()) {
//       RCLCPP_WARN(this->get_logger(), "Pose file does not exist. Please select a different command.");
//       return;
//     }
//     YAML::Node node = YAML::LoadFile(file_path);
//     std::vector<double> positions = node["position"].as<std::vector<double>>();

//     for (int i = 0; i < DOF_JOINTS; i++) {
//         desired_position[i] = positions[i];
//     }

//     //command_place(_can_handle);
//     //ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
//   }
// }

void AllegroNode485::libCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [%s]", msg->data.c_str());
    const std::string command = msg->data.c_str();

    CommandLength cmdLength;
    std::vector<uint8_t> cmd;

    auto itr = bhand_grasps.find(msg->data);
    if (itr != bhand_grasps.end()) {
        if (command.find("PosRead") == 0) 
            cmdLength = POSREAD;
        else
            cmdLength = COMMANDREAD;
         const auto& cmd = itr->second;
         sendData(fd, cmd.data(), cmd.size(), cmdLength);

    } else if (command.find("SavPos") == 0) {
        cmdLength = WRITEPOS;

        if (command.size() > 6 && isdigit(command[6])) {
            int index = command[6] - '0'; 
            if (index >= 1 && index <= 9) {
                cmd = {SAV_POS[0], static_cast<uint8_t>(index)}; 

                sendData(fd, cmd.data(), cmd.size(), cmdLength);
            } 
        }
    } else if (command.find("GoPos") == 0) {
        cmdLength = WRITEPOS;

        if (command.size() > 5 && isdigit(command[5])) {
            int index = command[5] - '0'; 
            if (index >= 1 && index <= 9) {
                cmd = {GO_POS[0], static_cast<uint8_t>(index)}; 

                sendData(fd, cmd.data(), cmd.size(), cmdLength);
            } 
        }
    }
     else {
        RCLCPP_WARN(this->get_logger(), "Unknown command type: [%s]", command.c_str());
        return; 
    }
}


void AllegroNode485::initController(const std::string &whichPort) {
  // Initialize RS-485 controller

  const char* portName = whichPort.c_str();//"/dev/ttyUSB0";
  int baudrate = getBaudrate(115200);
  fd = RS485init(portName, baudrate);
  
 // RCLCPP_INFO(this->get_logger(), "Allegro Hand RS-485 node start.");

  std::cout << "*************************************"<<std::endl;
  std::cout << "         RS-485 Grasp Method         "<<std::endl;
  std::cout << "-------------------------------------"<<std::endl;
  std::cout << "         Every command works.        "<<std::endl;
  std::cout << "*************************************"<<std::endl;
}

void AllegroNode485::doIt(bool polling) {
  auto this_node = std::shared_ptr<AllegroNode485>(this);

  if (polling) {
    RCLCPP_INFO(this->get_logger(), " Polling = true.");
    while (rclcpp::ok()) {
      rclcpp::spin_some(this_node);
    }
  }
  running = false;

  close(fd);
  RCLCPP_INFO(this->get_logger(), "RS-485 port closed.");
}

int main(int argc, char **argv) {
  auto clean_argv = rclcpp::init_and_remove_ros_arguments(argc, argv); 

  bool polling = true;

  AllegroNode485 allegro485;
  allegro485.doIt(polling);
}
