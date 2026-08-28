import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from xarmclient import XArm

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from xarmclient import XArm
from ourbeloved import wx250s_kinematics as k

import time

CARTESIAN_MODE = 0
JOINT_MODE = 1

class RobotDriverNode(Node):

    def __init__(self):
        super().__init__("robot_driver_node")
        self.goal_subscription = self.create_subscription(JointState, "joint_goal", self.goal_callback, 1)
        self.mode_subscription = self.create_subscription(Bool, "mode", self.mode_callback, 1)

        self.homing_subcription = self.create_subscription(Bool, "homing", self.homing_callback, 1)
        self.fire_subscription = self.create_subscription(Bool, "fire", self.fire_callback, 1)

        self.joint_state_publisher = self.create_publisher(JointState, "joint_state", 10)
        self.create_timer(0.05, self.publish_state)

        self.mode = JOINT_MODE
        self.xarm = XArm()
        self.xarm.grip(0) # Open gripper to begin

    def publish_state(self):
        msg = JointState()
        joint_tuple = self.xarm.get_joints()
        msg.position = [state * (3.14 / 180.0) for state in joint_tuple]
        msg.name = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.joint_state_publisher.publish(msg)

    def mode_callback(self, msg):
        self.mode = msg.data

    def goal_callback(self, msg):
        if not self.xarm.is_goal_valid(tuple(msg.position)) and self.mode == JOINT_MODE:
            self.xarm.set_joints(tuple(msg.position), "low_acc", [30,30,30,30,30,30]) # Velocities 30 to 45 degrees/sec
        elif not self.xarm.is_goal_valid(tuple(msg.position)) and self.mode == CARTESIAN_MODE:
            # Get the current physical position
            current_joints = self.xarm.get_joints()
            target_joints = msg.position
            
            # We found that using one speed meant the cartesian mode was extremely slow
            # So we need to change the speed depending on the distance it needs to travel
            
            # Calculate the absolute distance each joint needs to travel
            deltas = [abs(target_joints[i] - current_joints[i]) for i in range(6)]
            
            # Find the largest distance, to base our proportions on
            max_delta = max(deltas)
            
            # Calculate proportional speeds
            if max_delta > 0.1: # Prevent division by zero
                max_speed = 30.0 # Base speed limit
                
                # Scale each speed: (Joint Distance / Max Distance) * Max Speed
                proportional_speeds = [(d / max_delta) * max_speed for d in deltas]
                
                # Safety catch: Do not set speed to 0.0
                proportional_speeds = [max(s, 1.0) for s in proportional_speeds]
                
                self.xarm.set_joints(tuple(msg.position), "low_acc", proportional_speeds)
            else:
                # Movement is tiny, just send the position
                self.xarm.set_joints(tuple(msg.position))
        else:
            self.get_logger().warn("Joint goal is invalid")

    def fire_callback(self, msg):
        if msg.data == True:
            self.xarm.grip(1)
        else:
            self.xarm.grip(0)

    def homing_callback(self, msg):
        if msg.data == True:
            self.xarm.home()

    """def homing_callback(self, request, response):
        self.get_logger().info(f"Going Home")
        self.xarm.home()
        response.success = True
        return response"""

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = RobotDriverNode()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()