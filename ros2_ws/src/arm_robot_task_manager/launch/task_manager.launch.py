from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='open_manipulator_x', package_name='open_manipulator_moveit_config')
        .robot_description_semantic(
            str(Path('config') / 'open_manipulator_x' / 'open_manipulator_x.srdf'))
        .joint_limits(str(Path('config') / 'open_manipulator_x' / 'joint_limits.yaml'))
        .trajectory_execution(
            str(Path('config') / 'open_manipulator_x' / 'moveit_controllers.yaml'))
        .robot_description_kinematics(
            str(Path('config') / 'open_manipulator_x' / 'kinematics.yaml'))
        .to_moveit_configs()
    )

    task_manager = Node(
        package="arm_robot_task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ]
    )

    camera_config = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['--x', '0.35',
          '--y', '0.0',
          '--z', '1.15', ## conveyor world height:0.05, camera above conveyor:1.20, robot spawn height:0.10
          '--roll', '3.14159',
          '--pitch', '0.0',
          '--yaw', '0.0',
          '--frame-id', 'world',
          '--child-frame-id', 'overhead_camera/camera/rgbd_camera',
        ],
        output="screen",
    )

    return LaunchDescription([
        task_manager,
        camera_config
    ])