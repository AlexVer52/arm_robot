import gymnasium as gym
import numpy as np
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from arm_robot_interfaces.action import Grasp
from arm_robot_interfaces.srv import MoveHomePosition

import subprocess

class GraspActionClient(Node):
    def __init__(self):
        super().__init__('grasp_action_client')
        self._action_client = ActionClient(self, Grasp, 'arm_robot/grasp')

        self.latest_item_pose = None
        self.pose_update_count = 0

        self.item_pose_subscription = self.create_subscription(
            PoseStamped,
            "/arm_robot/item_pose",
            self.item_pose_callback,
            10,
        )

    def item_pose_callback(self, msg):
        q = msg.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.latest_item_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            yaw,
        ], dtype=np.float32)

        self.pose_update_count += 1

    def wait_for_item_pose(self, previous_count=None, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok():
            is_new = (
                self.latest_item_pose is not None
                and (
                    previous_count is None
                    or self.pose_update_count > previous_count
                )
            )

            if is_new:
                return self.latest_item_pose.copy()

            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out waiting for item pose")

            rclpy.spin_once(self, timeout_sec=0.05)

        raise RuntimeError("ROS was shut down")

    def send_goal(self, x, y, z, yaw):
        goal_msg = Grasp.Goal()
        goal_msg.target_pose.header.frame_id = "world"
        goal_msg.target_pose.pose.position.x = float(x)
        goal_msg.target_pose.pose.position.y = float(y)
        goal_msg.target_pose.pose.position.z = float(z)
        goal_msg.target_pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        goal_msg.target_pose.pose.orientation.w = float(np.cos(yaw / 2.0))

        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
           return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result.success

class MoveHomePositionClient(Node):
    def __init__(self):
        super().__init__('move_hope_postion_client')
        self.cli = self.create_client(MoveHomePosition, '/arm_robot/move_home_position')

    def send_request(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Move-home service is unavailable")

        request = MoveHomePosition.Request()

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            raise RuntimeError("Move-home service call failed")

        return future.result()

class GraspEnv(gym.Env):
    def __init__(self):
        super(GraspEnv, self).__init__()
        
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = gym.spaces.Box(low=np.array([-0.1, -0.1, -0.1, -3.14]), high=np.array([0.1, 0.1, 0.1, 3.14]), dtype=np.float32)
        # Example for using image as input:
        self.observation_space = gym.spaces.Box(low=np.array([-1.0, -1.0, 0.0, -3.14]), high=np.array([1.0, 1.0, 1.0, 3.14]), dtype=np.float32)

        self.current_observation = None
        self.grasp_success : bool = False

        self.grasp_action_client = GraspActionClient()
        self.move_home_client = MoveHomePositionClient()

    def grasp_result_callback(self, msg):
        self.grasp_success = msg.data

    def reset(self, seed = None, options = None):
        super().reset(seed=seed)
        cube_x, cube_y, cube_z, cube_yaw = self.reset_env()
        self.current_observation = self.get_observations()
        info = {}

        return self.current_observation.copy(), info

    def step(self, action):
        dx, dy, dz, dyaw = action

        grasp_x = self.current_observation[0] + dx
        grasp_y = self.current_observation[1] + dy
        grasp_z = self.current_observation[2] + dz
        grasp_yaw = self.current_observation[3] + dyaw

        success = self.grasp_action_client.send_goal(grasp_x, grasp_y, grasp_z, grasp_yaw)

        # Return observation, reward, done, info
        reward = float(self.compute_reward(success, action))
        observation = self.get_observations()

        terminated = bool(True)
        truncated = bool(False)

        info = {
            "success": success
        }
        return observation, reward, terminated, truncated, info

    def render(self, mode='human'):
        # Render the environment to the screen or other modes
        pass

    def close(self):
        self.grasp_action_client.destroy_node()
        self.move_home_client.destroy_node()
        pass

    def reset_env(self):
        x, y, z, yaw = self.move_random_cube()

        ## Reset Robot pose
        ## Cleaner: Use ROS for robot commands
        ## The clean solution is to expose a command interface from your task manager. => /arm_robot/command = "home" Then your C++ task manager receives it and runs:
        ##subprocess.run([
        ##    "ros2", "topic", "pub", "--once",
        ##    "/arm_robot/command",
        ##    "std_msgs/msg/String",
        ##    "{data: 'home'}"
        ##])
        response = self.move_home_client.send_request()

        if not response.success:
            raise RuntimeError(response.message)

        return x, y, z, yaw

    def move_random_cube(self):
        ## Randomize the cube position and orientation
        ## Later on, we can add variation of size, color, and shape of the cube to make the task more challenging
        r_min = 0.12
        r_max = 0.3

        r = np.sqrt(np.random.uniform(r_min**2, r_max**2))
        theta = np.random.uniform(-np.pi / 2, np.pi / 2)

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        z = 0.02
        yaw = np.random.uniform(-np.pi, np.pi)

        move_item = self.set_item_pose("item", x, y, z, yaw)

        if not move_item:
            raise RuntimeError("Could not move item in gazebo")

        return x, y, z, yaw

    def set_item_pose(self, name, x, y, z, yaw):
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)

        req = f""" 
            name: "{name}"
            position {{
              x: {x}
              y: {y}
              z: {z}
            }}
            orientation {{
              x: 0
              y: 0
              z: {qz}
              w: {qw}
            }}
            """

        result = subprocess.run(
            [
                "gz", "service",
                "-s", "/world/RLTraining/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "2000",
                "--req", req,
            ],
            text=True,
            capture_output=True,
        )

        if result.returncode != 0:
            print("Gazebo service failed:")
            print(result.stderr)
            return False

        print(result.stdout)
        return True

    def get_observations(self):
        previous_count = self.grasp_action_client.pose_update_count

        return self.grasp_action_client.wait_for_item_pose(
            previous_count=previous_count,
            timeout_sec=2.0,
        )

    def compute_reward(self, success, action):
        dx, dy, dz, dyaw = action
        reward = 0

        if success:
             reward += 10  # Reward for successful grasp
        else:
            reward -= 5  # Penalty for failed grasp
        
        ## Penalty for large corrections
        reward -= (abs(dx) + abs(dy) + abs(dz)) * 0.2
        reward -= abs(dyaw) * 0.05

        ## Later, implement reward based on cube angle and force to encourage better grasps
        return reward
