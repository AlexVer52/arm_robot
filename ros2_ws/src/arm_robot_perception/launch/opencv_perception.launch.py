from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navigation = Node(
        package="arm_robot_perception",
        executable="opencv_detect_node",
        name="opencv_detect_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        navigation
    ])