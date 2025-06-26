# allegro_hand_isaacsim_package



# Setup

## REAL2SIM

|**Instruction**|**Figure**|
|----------------------------------------------|----------------------|
| 1. Launch Isaac Sim & Check Extenstions(ROS2 Bridge)   | <img width="100%" height="100%" src="./asset/Check_extensions.png"> | 
| 2. Open AH_V5(3F)_REAL2SIM.usd file & Select ‘Yes’ on the warning message. | <img width="100%" height="100%" src="./asset/REAL2SIM_warning.png"> | 
| 3. Check Action Graph : <br> - **ROS Subscriber** <br> topic name : allegroHand_0/joint_states <br> - **Articulation Controller** <br> target prim : /World/allegro_hand | <img width="100%" height="100%" src="./asset/ROS_subscribe_joint_states.png"> |
| 4. Launch Allegro Hand V5 controller node |``` ros2 launch allegro_hand_controllers allegro_hand.launch.py ``` |
| 5. Press Play Button &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; &nbsp; &nbsp; &nbsp; &nbsp;| <img   width="100%" height="100%" src="./asset/REAL2SIM_play_3F.png">|


## SIM2REAL

|**Instruction**|**Figure**|
|------------------------|----------------------|
| 1. Launch Isaac Sim & Check Extenstions(ROS2 Bridge)  | <img width="100%" height="100%" src="./asset/Check_extensions.png"> | 
| 2. Open AH_V5(3F)_SIM2REAL.usd file| <img width="100%" height="100%" src="./asset/SIM2REAL.png"> | 
| 3. Check Action Graph : <br> - **ROS Publisher(Joint states)** <br> topic name : allegroHand_sim/joint_states <br> targetPrim : /World/allegro_hand/root_joint <br> - **ROS Publisher(Contact sensor)** <br> topic name : allegroHand_sim/contact_sensors <br> - **Articulation Controller** <br> target prim : /World/allegro_hand/root_joint | <img width="100%" height="100%" src="./asset/SIM2REAL_actiongraph.png"> |
| 4. Launch Allegro Hand V5 controller node with Isaac arguments | ``` ros2 launch allegro_hand_controllers allegro_hand.launch.py ISAAC:=true ``` |
| 5. Press Play Button & Control the hand joint changing joint_positions_array of ActionGraph| <img   width="100%" height="100%" src="./asset/SIM2REAL_play_3F.png"> |

