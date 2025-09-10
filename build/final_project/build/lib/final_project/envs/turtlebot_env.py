# final_project/envs/turtlebot_env.py
import rclpy
from rclpy.node import Node
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from launch.substitutions import LaunchConfiguration
import time

class Turtlebot3GymEnv(gym.Env, Node):
    """Gym environment for TurtleBot3 in ROS2 + Gazebo with reset to launch x_pose/y_pose."""

    def __init__(self,
                 node_name='turtlebot3_env_rl',
                 scan_topic='/scan',
                 odom_topic='/odom',
                 cmd_topic='/cmd_vel',
                 lidar_dims=360,
                 max_env_steps=500,
                 x_pose=-2.0,
                 y_pose=-0.5):

        rclpy.init(args=None)
        Node.__init__(self, node_name)

        # ROS2 publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # Robot state
        self.robot_position = np.array([x_pose, y_pose])
        self.robot_orientation = 0.0
        self.lidar_data = np.zeros(lidar_dims)
        self.collision = False

        # Start position from launch
        self.start_position = np.array([x_pose, y_pose])

        # Goal
        self.goal_position = np.array([1.636, 0.097])  # fixed goal
        self.prev_dist_to_goal = None

        # Env parameters
        self.max_steps = max_env_steps
        self.current_step = 0
        self.action_space = spaces.Box(low=np.array([-0.22, -2.84]),
                                       high=np.array([0.22, 2.84]),
                                       dtype=np.float32)  # linear_x, angular_z

        # Observation: LIDAR + distance & angle to goal
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(lidar_dims + 3,), dtype=np.float32)

        # Wait a little for topics to initialize
        time.sleep(1.0)

    # ---------------- ROS2 callbacks ----------------
    def odom_callback(self, msg):
        self.robot_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.lidar_data = np.array(msg.ranges)
        if np.min(self.lidar_data) < 0.15:
            self.collision = True
        else:
            self.collision = False

    # ---------------- Gym interface ----------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        self.collision = False
        # Reset robot to start position from launch file
        self.robot_position = self.start_position.copy()
        self.prev_dist_to_goal = np.linalg.norm(self.robot_position - self.goal_position)
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        self.current_step += 1
        # Clip actions
        linear_x = np.clip(action[0], -0.22, 0.22)
        angular_z = np.clip(action[1], -2.84, 2.84)

        # Publish command
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

        # Small delay for physics update
        time.sleep(0.1)

        # Observation
        obs = self._get_obs()

        # Reward
        dist_to_goal = np.linalg.norm(self.robot_position - self.goal_position)
        reward = (self.prev_dist_to_goal - dist_to_goal) * 10.0
        self.prev_dist_to_goal = dist_to_goal

        done = False
        if dist_to_goal < 0.2:
            reward += 100.0
            done = True
        if self.collision:
            reward -= 100.0
            done = True
        if self.current_step >= self.max_steps:
            done = True

        return obs, reward, done, False, {}

    def _get_obs(self):
        vec_to_goal = self.goal_position - self.robot_position
        dist_to_goal = np.linalg.norm(vec_to_goal)
        angle_to_goal = np.arctan2(vec_to_goal[1], vec_to_goal[0]) - self.robot_orientation
        angle_to_goal = np.arctan2(np.sin(angle_to_goal), np.cos(angle_to_goal))
        obs = np.concatenate([self.lidar_data, np.array([dist_to_goal, angle_to_goal, 0.0])])
        return obs.astype(np.float32)
