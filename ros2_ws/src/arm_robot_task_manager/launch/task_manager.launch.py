from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    task_manager = Node(
        package="arm_robot_task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        task_manager
    ])