from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition,UnlessCondition
import os
import getpass

def generate_launch_description():
    allegro_hand_controllers_share = get_package_share_directory('allegro_hand_controllers')
    allegro_hand_moveit_share = get_package_share_directory('allegro_hand_moveit')
    # Declare launch argument
    declare_visualize_arg = DeclareLaunchArgument(
        'VISUALIZE',
        default_value='false',
        description='Flag to enable/disable visualization'
    )
	
    declare_moveit_arg = DeclareLaunchArgument(
        'MOVEIT',
        default_value='false',
        description='Flag to enable/disable moveit2'
    )
	
    declare_gui_arg = DeclareLaunchArgument(
        'GUI',
        default_value='false',
        description='Flag to enable/disable GUI'
    )

    declare_sim_arg = DeclareLaunchArgument(
        'ISAAC',
        default_value='false',
        description='Flag to enable/disable ISAAC SIM2REAL'
    )

    declare_polling_arg = DeclareLaunchArgument(
        'POLLING',
        default_value='true',
        description='true, false for polling the CAN communication'
    )
    
    declare_can_device_arg = DeclareLaunchArgument(
    	'CAN_DEVICE', 
    	default_value='can0',
    	description='Specify CAN port for control multi devices'
    )
    
    declare_num_arg = DeclareLaunchArgument(
    	'NUM',
    	default_value='0',
    	description='Specify AH num for control multi devices'
    )
	
    declare_rs485_arg = DeclareLaunchArgument(
    	'RS485',
    	default_value= 'false',
    	description='Specify AH communication: RS485 or CAN'
    )
	
    declare_port_arg = DeclareLaunchArgument(
    	'PORT',
    	default_value= '/dev/ttyUSB0',
    	description='Specify RS-485 port'
    )
    
    def setup_can(context):
    	can_port = context.launch_configurations['CAN_DEVICE']
    	commands = [
        	f"sudo ip link set {can_port} down",
        	f"sudo ip link set {can_port} type can bitrate 1000000",
        	f"sudo ip link set {can_port} up"
    	]
    
    	while True:
        	password = getpass.getpass('Enter sudo password: ')
        	success = True
        
        	for cmd in commands:
            		result = os.system(f'echo "{password}" | sudo -S {cmd}')
            		if result != 0:
                		print(f"Command failed: {cmd}")
                		success = False
                		break
        
        	if success:
            		print(f'{can_port} setup completed')
            		break
        	else:
            		print(f'{can_port} setup failed. Please try again.')
    
    	return []  
        
    urdf_path = PythonExpression([
        '"', allegro_hand_controllers_share, '/urdf/allegro_hand_description_3f.urdf"'
    ])

    return LaunchDescription([
        declare_visualize_arg,
        declare_polling_arg,
        declare_can_device_arg,
        declare_num_arg,
        declare_moveit_arg,
        declare_gui_arg,
        declare_rs485_arg,
        declare_port_arg,
        declare_sim_arg,
        OpaqueFunction(function=setup_can,
					   condition=UnlessCondition(LaunchConfiguration('RS485'))),
        Node(
            package='allegro_hand_controllers',
            executable='allegro_node_grasp',
            output='screen',
			condition=UnlessCondition(LaunchConfiguration('RS485')),
            parameters=[{'comm/CAN_CH': LaunchConfiguration('CAN_DEVICE')}],
            arguments=[LaunchConfiguration('POLLING')],
            remappings=[('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"])),
            		    ('allegroHand/joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
				        ('allegroHand/joint_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_cmd'"])),
					    ('allegroHand/envelop_torque',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/envelop_torque'"])),
					    ('forcechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/force_chg'"])),
					    ('timechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/time_chg'"]))
            ]
        ),
        Node(
            package='allegro_hand_controllers',
            executable='allegro_node_485',
            output='screen',
			condition=IfCondition(LaunchConfiguration('RS485')),
			parameters=[{'port_info/which_port': LaunchConfiguration('PORT')}],
            arguments=[LaunchConfiguration('POLLING')],
			remappings=[('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"]))]
        ),		
        Node(
            package='robot_state_publisher',
            output='log',
            arguments=['--ros-args', '--log-level', 'WARN'],
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
            remappings=[
                ('tf', PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/tf'"])),
                ('joint_states',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_states'"])),
                ('robot_description', 'allegro_hand_description')
            ]
        ),
        # Include the allegro_viz.launch.py file if VISUALIZE is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_controllers_share, 'launch', 'allegro_viz.launch.py')),
            condition=IfCondition(LaunchConfiguration('VISUALIZE')),
            launch_arguments={'NUM': LaunchConfiguration('NUM')}.items()
        ),
		# Include the demo.launch.py file if MOVEIT is true       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(allegro_hand_moveit_share, 'launch', 'demo.launch.py')),
            condition=IfCondition(LaunchConfiguration('MOVEIT'))
        ),
	    Node(
            package='allegro_hand_gui',
            executable='allegro_hand_gui_node',
            output='screen',
			remappings=[('allegroHand/lib_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/lib_cmd'"])),
				        ('forcechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/force_chg'"])),
				        ('timechange',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/time_chg'"]))],		
            condition=IfCondition(LaunchConfiguration('GUI'))
        ),
        Node(
            package='allegro_hand_isaacsim',
            executable='allegro_hand_sim2real',
            output='screen',
			remappings=[('allegroHand/joint_cmd',PythonExpression(["'allegroHand_",LaunchConfiguration('NUM'),"/joint_cmd'"]))],		
            condition=IfCondition(LaunchConfiguration('ISAAC'))
        )
        
    ])
