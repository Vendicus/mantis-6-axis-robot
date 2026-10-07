import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
#from ros_gz_sim import GazeboLauncher, GazeboBridge
import yaml

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )

    controllers_yaml_path = os.path.join(
    get_package_share_directory("mantis_moveit"),
    "config",
    "moveit_controllers.yaml"
    )

    with open(controllers_yaml_path, 'r') as f:
        controllers_config = yaml.safe_load(f)

    controllers_config_wrapped = {
        "moveit_simple_controller_manager": controllers_config
    }

    moveit_config = (
        MoveItConfigsBuilder("mantis", package_name="mantis_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("mantis_description"),
            "URDF",
            "robot.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/mantis.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {'use_sim_time': is_sim},
                    {'publish_robot_description_semantic': True},
                    {
                        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                    },
                    controllers_config_wrapped
                ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    pkg_name = "mantis_bringup"
    xml_file = "mantis_gazebo.launch.xml"
    
    xml_launch_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'launch',
        xml_file
    )
    
    xml_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(xml_launch_file_path)
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("mantis_moveit"),
            "config",
            "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    
    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node, 
            rviz_node,
            xml_launch
        ]
    )