import os
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import TimerEvent

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()

    joint_controllers_file = os.path.join(
        get_package_share_directory('mantis_moveit_setup_assistant_config'), 'config', 'mantis_controllers.yaml'
    )
    gazebo_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    world_file_path = os.path.join(
        get_package_share_directory('mantis_bringup'), 'world', 'world.sdf'
    )

    moveit_config = (
        MoveItConfigsBuilder("mantis_robot", package_name="mantis_moveit_setup_assistant_config")
        .robot_description(file_path="config/mantis.urdf.xacro")
        .robot_description_semantic(file_path="config/mantis.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # --------------------------------- detach commands for gripper on start -------------
    detach_box_1 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '5', '-r', '10', '/box/detach', 'std_msgs/msg/Empty', '{}']
    )

    detach_box_2 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '5', '-r', '10', '/box1/detach', 'std_msgs/msg/Empty', '{}']
    )

     # --------------------------------- gazebo params on launch -------------    
    workspace_share_dir = os.path.join(get_package_prefix("mantis_description"), "share")
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=workspace_share_dir
    )

    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='1.07', description='Z position of the robot')

    sim_time_param = {"use_sim_time": True}

    gripper_server_node = Node(
        package='mantis_commander_py',          
        executable='detach_grip_service',
        name='gripper_server',
        output='screen',                     
        parameters=[
            sim_time_param
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gz_args": "-r " + world_file_path,
        }.items(),
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("mantis_moveit_setup_assistant_config"),
        "config",
        "moveit_gazebo.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_time_param
        ],
    )

    # spawn the robot
    spawn_the_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mantis',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
    )

    # bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/box/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/box/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/box1/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/box1/detach@std_msgs/msg/Empty]gz.msgs.Empty",
        ],
        output="screen",
    )

    # --- WARNING ----  
    # GAZEBO PLUGIN uses its own controller and doesn't requier controller manager

    # Robot state publisher + sim_time_param
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, sim_time_param],
        output='screen'
    )

    # use time synchronization 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    config_dict = moveit_config.to_dict()
    config_dict.update(sim_time_param)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )


    # ---------------- Delay start of controllers (after spawning robot in simulation) ----------
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_the_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_trajectory_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=arm_trajectory_controller_spawner,
            on_start=[gripper_controller_spawner],
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    delayed_detach_1 = TimerAction(period=5.0, actions=[detach_box_1])
    delayed_detach_2 = TimerAction(period=5.0, actions=[detach_box_2])

    # Launch Description
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(spawn_the_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(move_group_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(gripper_server_node)
    
    # Section for delayed actions
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_rviz_node)
    ld.add_action(delayed_detach_1)
    ld.add_action(delayed_detach_2)

    return ld