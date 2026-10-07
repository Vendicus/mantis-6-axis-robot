import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mantis_description = get_package_share_directory('mantis_description')
    mantis_description_share = get_package_prefix('mantis_description')
    gazebo_ros_dir = get_package_share_directory('ros_gz_sim')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        mantis_description, 'URDF', 'robot.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    model_path = os.path.join(mantis_description, "meshes")
    model_path = os.path.join(mantis_description_share, "share")

    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gz_sim.launch.py')
        )
    )


    spawn_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=[
                                    '-topic', 'robot_description',
                                  ],
                        output='screen'
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        robot_state_publisher_node,
        spawn_robot
    ])