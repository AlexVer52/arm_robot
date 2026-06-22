from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from arm_robot_rl_training.env.grasp_env import GraspEnv
import rclpy

rclpy.init()
env = None
try:
    env = Monitor(
        GraspEnv(),
        filename="grasp_training_monitor.csv",
        info_keywords=("success",),
    )
    model = PPO("MlpPolicy", env, verbose=1, n_steps=20, batch_size=20, n_epochs=5, device="cpu")
    model.learn(total_timesteps=20)
    model.save("grasp_ppo")
finally:
    if env is not None:
        env.close()
    if rclpy.ok():
        rclpy.shutdown()
