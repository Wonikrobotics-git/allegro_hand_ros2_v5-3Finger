// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Author: Hibo (sh-yang@wonikrobotics.com)

#include "allegro_node.h"
#include "allegro_hand_driver/AllegroHandDrv.h"


std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0"
        };


AllegroNode::AllegroNode(const std::string nodeName, bool sim /* = false */)
  : Node(nodeName)
{
  mutex = new std::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position[i] = 0.0;
    current_velocity[i] = 0.0;
  }


  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    declare_parameter("comm/CAN_CH", "can0");
    auto can_ch = this->get_parameter("comm/CAN_CH").as_string();
    if (canDevice->init(can_ch)) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = get_clock()->now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(DESIRED_STATE_TOPIC, 1, // queue size
                                 std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));
  time_sub = this->create_subscription<std_msgs::msg::Float32>("timechange", 1, // queue size
                                 std::bind(&AllegroNode::ControltimeCallback, this, std::placeholders::_1));
  force_sub = this->create_subscription<std_msgs::msg::Float32>("forcechange", 1, // queue size
                                 std::bind(&AllegroNode::GraspforceCallback, this, std::placeholders::_1));

}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
  rclcpp::shutdown();
}

void AllegroNode::desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  desired_joint_state = *msg;
  mutex->unlock();
}

void AllegroNode::ControltimeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    motion_time = msg->data;
    RCLCPP_INFO(this->get_logger(), "Setting motion time to %.3f.", motion_time);
    pBHand->SetMotiontime(motion_time);
}

void AllegroNode::GraspforceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    force_get = msg->data;
    RCLCPP_INFO(this->get_logger(), "Setting grasping force to %.3f.", force_get);
}

void AllegroNode::publishData() {
  // current position, velocity and effort (torque) published
  current_joint_state.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i];
    current_joint_state.velocity[i] = current_velocity[i];
    current_joint_state.effort[i] = desired_torque[i];
  }
  joint_state_pub->publish(current_joint_state);
}

void AllegroNode::updateController() {

  // Calculate loop time;
  tnow = get_clock()->now();
  dt = 1e-9 * (tnow - tstart).nanoseconds();//ALLEGRO_CONTROL_TIME_INTERVAL;//
  //printf("%lf\n",dt);
  // When running gazebo, sometimes the loop gets called *too* often and dt will
  // be zero. Ensure nothing bad (like divide-by-zero) happens because of this.
  if(dt <= 0) {
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("allegro_node"), *get_clock(), 1000, "AllegroNode::updateController dt is zero.");
    return;
  }

  tstart = tnow;


  if (canDevice)
  {
    // try to update joint positions through CAN comm:
    lEmergencyStop = canDevice->readCANFrames();

    // check if all positions are updated:
    if (lEmergencyStop == 0 && canDevice->isJointInfoReady())
    {
      // update joint positions:
      canDevice->getJointInfo(current_position);

      OperatingMode = 0;

      if (OperatingMode == 0) {
        if ((fingertip_sensor[0] + fingertip_sensor[1] + fingertip_sensor[2]) > 200)
          f[0] = f[1] = f[2] = force_get;//10.0f;
        else
          f[0] = f[1] = f[2] = 5.0f;
      }


      // calculate control torque:
      computeDesiredTorque();

      // set & write torque to each joint:
      canDevice->setTorque(desired_torque);
      lEmergencyStop = canDevice->writeJointTorque();

      // reset joint position update flag:
      canDevice->resetJointInfoReady();

      // publish joint positions to ROS topic:
      publishData();

      frame++;
    }

  }

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    RCLCPP_ERROR(rclcpp::get_logger("allegro_node"),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
    rclcpp::shutdown();
  }
}

void AllegroNode::timerCallback() {
  updateController();
}

using namespace std::chrono_literals; 

rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  auto timer = this->create_wall_timer(1ms, std::bind(&AllegroNode::timerCallback, this));
  return timer;
}
