import os
import subprocess
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    kuka_description = get_package_share_directory("kuka_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(kuka_description, "urdf", "kr10r1420.urdf.xacro"),
        description="Absolute path to robot URDF"
    )

    # ----------------------------------------------------------
    #  SET GAZEBO RESOURCE PATH FOR MODELS
    # ----------------------------------------------------------
    models_path = "/home/sourav/kuka_ws/src/models"
    kuka_src_path = "/home/sourav/kuka_ws/src"
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{kuka_src_path}:{models_path}"
    )

    # ----------------------------------------------------------
    #  ROBOT DESCRIPTION
    # ----------------------------------------------------------
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    # Build the xacro command with sed to convert package:// to file:// URIs
    xacro_file = os.path.join(kuka_description, "urdf", "kr10r1420.urdf.xacro")
    
    # Generate robot description by running xacro and sed
    xacro_cmd = f"xacro {xacro_file} is_ignition:={is_ignition}"
    xacro_result = subprocess.run(xacro_cmd, shell=True, capture_output=True, text=True)
    robot_urdf = xacro_result.stdout
    
    # Replace package:// URIs with file:// URIs
    robot_urdf = robot_urdf.replace("package://kuka_description/", f"file://{kuka_description}/")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_urdf,
            "use_sim_time": True
        }]
    )

    # ----------------------------------------------------------
    #  LAUNCH GZ SIM
    # ----------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", [" -v 4 -r empty.sdf"])]
    )

    # ----------------------------------------------------------
    #  SPAWN ROBOT
    # ----------------------------------------------------------
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "kr10r1420",
            "-x", "0.5", "-y", "0.0", "-z", "1.0"
        ],
    )

    # ----------------------------------------------------------
    #  BRIDGE
    # ----------------------------------------------------------
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
    )

    # ----------------------------------------------------------
    #  SPAWN TABLES
    # ----------------------------------------------------------
    table_path = "/home/sourav/kuka_ws/src/models/Table/model.sdf"

    def spawn_table(name, x, y, z):
        return Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-file", table_path,
                "-name", name,
                "-x", str(x), "-y", str(y), "-z", str(z)
            ],
        )

    table_nodes = [
        spawn_table("table1", 1.0, 0.0, 0.0),
        spawn_table("table2", 2.5, 0.0, 0.0),
        spawn_table("table3", 1.0, -0.8, 0.0),
        spawn_table("table4", 1.0,  0.8, 0.0),
        spawn_table("table5", 2.5,  0.8, 0.0),
        spawn_table("table6", 2.5, -0.8, 0.0),
    ]

    # ----------------------------------------------------------
    #  FINAL LAUNCH DESCRIPTION
    # ----------------------------------------------------------
    return LaunchDescription([
        model_arg,
        gz_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        *table_nodes
    ])
