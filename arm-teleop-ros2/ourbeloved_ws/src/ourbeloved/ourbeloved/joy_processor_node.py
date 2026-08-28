import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R

from ourbeloved import wx250s_kinematics as k

import time

CARTESIAN_MODE = False
JOINT_MODE = True

class JoyProcessorNode(Node):

    def __init__(self):
        super().__init__("joy_processor_node")
        self.controller_subscription = self.create_subscription(Joy, "joy", self.listener_callback, 1)
        self.joint_subscription = self.create_subscription(JointState, "joint_state", self.joint_sub_callback, 1)
        
        # self.homing_client = self.create_client(Trigger, "homing")
        self.goal_publisher = self.create_publisher(JointState, "joint_goal", 10)
        self.homing_publisher = self.create_publisher(Bool, "homing", 10)
        self.fire_publisher = self.create_publisher(Bool, "fire", 10)
        self.ee_pose_publisher = self.create_publisher(PoseStamped, "ee_pose", 10)
        self.mode_publisher = self.create_publisher(Bool, "mode", 10)
        
        self.create_timer(0.05, self.control_loop)
        self.mode = JOINT_MODE # If 0, will be cartesian, if 1, will be joint mode

        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0
        self.d_x = 0
        self.d_y = 0

        self.current_joints = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.speed_control = 8

        self.last_target_joints = None
        self.is_moving = False

        self.fire = False

    def joint_sub_callback(self, msg):
        for i in range(6):
            self.current_joints[i] = msg.position[i] * (180/3.14)

    def listener_callback(self, msg):        
        """if msg.buttons[10] == 1:
            request = Trigger.Request()
            response = self.homing_client.call_async(request)
            self.get_logger().info("Going home")"""
        
        # Go home
        if msg.buttons[10] == 1:
            homing_msg = Bool()
            homing_msg.data = True
            self.homing_publisher.publish(homing_msg)
            self.last_target_joints = None # Reset the last target joints

        # Fire
        if self.fire == False:
            if msg.buttons[7] == 1:
                self.fire = True

                fire_msg = Bool()
                fire_msg.data = True
                self.fire_publisher.publish(fire_msg)
        if self.fire == True:
            if msg.buttons[7] == 0:
                self.fire = False

                fire_msg = Bool()
                fire_msg.data = False
                self.fire_publisher.publish(fire_msg)

        # Get axes
        self.left_x = msg.axes[0]
        self.left_y = msg.axes[1]
        self.right_x = msg.axes[3]
        self.right_y = msg.axes[4]
        self.d_x = msg.axes[6]
        self.d_y = msg.axes[7]

        # Change mode
        if msg.buttons[8] == 1:
            self.mode = CARTESIAN_MODE

            mode_msg = Bool()
            mode_msg.data = CARTESIAN_MODE
            self.mode_publisher.publish(mode_msg)

            self.get_logger().info("Changed to Cartesian Mode")
        if msg.buttons[9] == 1:
            self.mode = JOINT_MODE

            mode_msg = Bool()
            mode_msg.data = JOINT_MODE
            self.mode_publisher.publish(mode_msg)

            self.get_logger().info("Changed to Joint Mode")

    def control_loop(self):
        ee_pose_msg = PoseStamped()
        ee_htm, _ = k.fk(self.current_joints)

        ee_pose_msg.pose.position.x = ee_htm[0, 3]/1000.0
        ee_pose_msg.pose.position.y = ee_htm[1, 3]/1000.0
        ee_pose_msg.pose.position.z = ee_htm[2, 3]/1000.0

        rotation_matrix = ee_htm[:, 0:3]
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()
        ee_pose_msg.pose.orientation.x = quaternion[0]
        ee_pose_msg.pose.orientation.y = quaternion[1]
        ee_pose_msg.pose.orientation.z = quaternion[2]
        ee_pose_msg.pose.orientation.w = quaternion[3]

        ee_pose_msg.header.frame_id = "base"

        self.ee_pose_publisher.publish(ee_pose_msg)

        # Check if the user is holding the sticks
        sticks_active = not (abs(self.left_x) < 0.1 and abs(self.left_y) < 0.1 and 
                             abs(self.right_x) < 0.1 and abs(self.right_y) < 0.1 and 
                             abs(self.d_x) < 0.1 and abs(self.d_y) < 0.1)

        # If sticks are released, stop the robot immediately
        if not sticks_active:
            if self.is_moving:
                # Send the CURRENT joints as a goal to halt it instantly
                msg = JointState()
                msg.name = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
                
                if self.last_target_joints is not None:
                    for i in range(6):
                        # If the robot was in the middle of moving, halt it
                        error = self.last_target_joints[i] - self.current_joints[i]
                        if abs(error) > 2.0:
                            self.last_target_joints[i] = self.current_joints[i] + (error * 0.15)
                    msg.position = list(self.last_target_joints)
                else:
                    msg.position = list(self.current_joints)

                self.goal_publisher.publish(msg)
                
                self.is_moving = False
            return # Abort!
        
        self.is_moving = True

        # Prevent sending new commands until the arm catches up
        if self.last_target_joints is not None:
            # Calculate how far the arm is from the last command sent
            error = sum(abs(c - t) for c, t in zip(self.current_joints, self.last_target_joints))
            if error > 15.0:  # If it's more than 15 degrees behind, wait
                return

        if self.mode == CARTESIAN_MODE:
            # Initialise ideal target state
            if self.last_target_joints is None:
                self.last_target_joints = list(self.current_joints)

            # Get current xyz
            htm, _ = k.fk(self.last_target_joints)

            current_x = htm[0, 3]/1000.0
            current_y = htm[1, 3]/1000.0
            current_z = htm[2, 3]/1000.0

            # Get axes
            controller_x_axis = self.left_y/100 # Left stick x
            controller_y_axis = self.left_x/100 # Left stick y
            controller_z_axis = self.right_y/100 # Right stick y

            # Add to htm
            goal_x = current_x + controller_x_axis
            goal_y = current_y + controller_y_axis
            goal_z = current_z + controller_z_axis

            # Use inverse kinematics to get new joints
            goal_htm = htm.copy()
            goal_htm[0, 3] = goal_x * 1000
            goal_htm[1, 3] = goal_y * 1000
            goal_htm[2, 3] = goal_z * 1000

            goal_joints = k.ik(self.last_target_joints, goal_htm)

            # Check if these are valid
            if goal_joints is None:
                self.get_logger().warn("IK falied to converge")
            else:
                msg = JointState()
                msg.name = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
                msg.position = [float(j) for j in goal_joints]

                self.last_target_joints = msg.position # Track the target
                self.goal_publisher.publish(msg)

        elif self.mode == JOINT_MODE:
            joy_list = [self.left_x, self.left_y, self.right_y, -self.right_x, self.d_y, -self.d_x]
            
            if self.last_target_joints is None:
                self.last_target_joints = list(self.current_joints)
            
            new_joints = list(self.last_target_joints)

            for i in range(6):
                if abs(joy_list[i]) > 0.1:
                    new_joints[i] += self.speed_control * joy_list[i]
            
            # ONLY SEND COMMAND IF PREVIOUS GOAL HAS BEEN REACHED, THAT WAY THERE IS NO DRIFT/BACKLOG        
            if sticks_active:
                msg = JointState()
                msg.name = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
                msg.position = new_joints

                self.last_target_joints = msg.position # Track the target
                self.goal_publisher.publish(msg)

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = JoyProcessorNode()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()