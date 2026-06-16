#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, inverse_matrix


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_motion_planner")
        self.declare_parameter("look_ahead_distance", 0.5)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.look_ahead_distance = self.get_parameter("look_ahead_distance").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.carrot_pose_pub = self.create_publisher(PoseStamped, "/pure_pursuit/carrot", 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.global_plan = None

    def path_callback(self, path: Path):
        self.global_plan = path

    def control_loop(self):
        if not self.global_plan or not self.global_plan.poses:
            return
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception as ex:
            self.get_logger().warn(f"Could not transform: {ex}")
            return
        # self.get_logger().info(f"frame_id Robot Pose: {robot_pose_transform.header.frame_id}")
        # self.get_logger().info(f"frame_id Global Plan: {self.global_plan.header.frame_id}")
        if not self.transform_plan(robot_pose_transform.header.frame_id):
            self.get_logger().error("Unable to transform Plan in robot's frame")
            return
        
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = robot_pose_transform.header.frame_id
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation
        carrot_pose: PoseStamped = self.get_carrot_pose(robot_pose)

        dx = carrot_pose.pose.position.x - robot_pose.pose.position.x
        dy = carrot_pose.pose.position.y - robot_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy*dy)

        if distance <= 0.1:
            self.get_logger().info("Goal Reached!")
            self.global_plan.poses.clear()
            return
        self.carrot_pose_pub.publish(carrot_pose)
        robot_tf = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
        ])
        robot_tf[0][3] = robot_pose.pose.position.x
        robot_tf[1][3] = robot_pose.pose.position.y
        carrot_pose_tf = quaternion_matrix([
            carrot_pose.pose.orientation.x,
            carrot_pose.pose.orientation.y,
            carrot_pose.pose.orientation.z,
            carrot_pose.pose.orientation.w
        ])
        carrot_pose_tf[0][3] = carrot_pose.pose.position.x
        carrot_pose_tf[1][3] = carrot_pose.pose.position.y
        
        carrot_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), carrot_pose_tf)
        carrot_pose_robot = PoseStamped()
        carrot_pose_robot.pose.position.x = carrot_pose_robot_tf[0][3]
        carrot_pose_robot.pose.position.y = carrot_pose_robot_tf[1][3]
        carrot_pose_robot.pose.position.z = carrot_pose_robot_tf[2][3]
        quaternion = quaternion_from_matrix(carrot_pose_robot_tf)
        carrot_pose_robot.pose.orientation.x = quaternion[0]
        carrot_pose_robot.pose.orientation.y = quaternion[1]
        carrot_pose_robot.pose.orientation.z = quaternion[2]
        carrot_pose_robot.pose.orientation.w = quaternion[3]

        curvature = self.get_curvature(carrot_pose_robot.pose)
        cmd_vel = Twist()
        cmd_vel.linear.x = self.max_linear_velocity
        cmd_vel.angular.z = curvature * self.max_angular_velocity

        self.cmd_pub.publish(cmd_vel)

    #Top = Tom * Tmp
    def transform_plan(self, frame):
        if self.global_plan.header.frame_id == frame:
            return True 
        #global_path in map. Need convert to odom
        try:
            #get pose from tf
            # #Muốn từ Map sang Odom thì phải dịch bao nhiêu và quay bao nhiêu    
            transform = self.tf_buffer.lookup_transform(frame, self.global_plan.header.frame_id, rclpy.time.Time())
        except Exception as ex:
            self.get_logger().error(f"Couldn't transform plan from frame {self.global_plan.header.frame_id} to frame {frame}: {ex}")
            return False
        #theta được biểu diễn bằng quaternion xyzw, nằm trong orientation
        #rotation: robot phải quay bao nhiêu
        transform_matrix = quaternion_matrix([transform.transform.rotation.x, 
                                              transform.transform.rotation.y, 
                                              transform.transform.rotation.z, 
                                              transform.transform.rotation.w])
        #translation: robot phải dịch bao nhiêu
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y
        
        for pose in self.global_plan.poses:
            pose_matrix = quaternion_matrix([pose.pose.orientation.x, 
                                              pose.pose.orientation.y, 
                                              pose.pose.orientation.z, 
                                              pose.pose.orientation.w])
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y
            transformed_pose = concatenate_matrices(pose_matrix, transform_matrix)
            [pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] = translation_from_matrix(transformed_pose)
            pose.header.frame_id = frame

        self.global_plan.header.frame_id = frame
        return True
    
    def get_carrot_pose(self, robot_pose: PoseStamped):
        carrot_pose = self.global_plan.poses[-1]
        for pose in reversed(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy*dy)
            if distance > self.look_ahead_distance:
                carrot_pose = pose
            else:
                break
        return carrot_pose

    def get_curvature(self, carrot_pose: Pose):
        L = carrot_pose.position.x ** 2+ carrot_pose.position.y ** 2
        if L > 0.001:
            return 2.0 * carrot_pose.position.y / L
        else:
            return 0.0


def main():
    rclpy.init()
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()