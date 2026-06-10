import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("sort_robo_description"),
                    "urdf",
                    "panda.urdf.xacro",
                ),
                " is_sim:=",
                is_sim,
                " is_ignition:=True" # remember to make it according to the Gazebo version
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                "robot_description_topic": "/robot_description",
            },
            os.path.join(
                get_package_share_directory("sort_robo_controller"),
                "config",
                "controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    # Spawner nodes with longer timeout
    def create_spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
            output="screen",
        )

    joint_state_broadcaster_spawner = create_spawner("joint_state_broadcaster")
    arm_controller_spawner = create_spawner("arm_controller")
    gripper_controller_spawner = create_spawner("gripper_controller")

    return LaunchDescription(
        [
            is_sim_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )