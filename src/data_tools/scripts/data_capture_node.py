#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import cv2
import numpy as np
import argparse
import yaml

import rospy
from sensor_msgs.msg import JointState, Image, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

try:
    from data_msgs.msg import Gripper
except ImportError:
    rospy.logwarn("data_msgs.msg.Gripper not available, gripper data capture will be disabled")
    Gripper = None

import ros_numpy


class DataCaptureNode:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.episode_index = args.episode_index
        self.dataset_dir = args.dataset_dir
        
        # Create episode directory
        self.episode_dir = os.path.join(self.dataset_dir, f"episode{self.episode_index}")
        if not os.path.exists(self.episode_dir):
            os.makedirs(self.episode_dir)
        
        # Create data directories
        self.create_data_directories()
        
        # Initialize subscribers
        self.init_subscribers()
        
        rospy.loginfo("Data capture node initialized for episode %d", self.episode_index)
        rospy.loginfo("Data will be saved to: %s", self.episode_dir)
        if 'image_top' in self.args.camera_color_names:
            idx = self.args.camera_color_names.index('image_top')
            rospy.loginfo("image_top interface configured at topic: %s", self.args.camera_color_topics[idx])

    def create_data_directories(self):
        """Create all necessary data directories for the episode"""
        # Camera directories
        for name in self.args.camera_color_names:
            dir_path = os.path.join(self.episode_dir, f"camera/color/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        for name in self.args.camera_depth_names:
            dir_path = os.path.join(self.episode_dir, f"camera/depth/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        for name in self.args.camera_point_cloud_names:
            dir_path = os.path.join(self.episode_dir, f"camera/pointCloud/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        # Arm directories
        for name in self.args.arm_joint_state_names:
            dir_path = os.path.join(self.episode_dir, f"arm/jointState/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        for name in self.args.arm_end_pose_names:
            dir_path = os.path.join(self.episode_dir, f"arm/endPose/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        # Localization directories
        for name in self.args.localization_pose_names:
            dir_path = os.path.join(self.episode_dir, f"localization/pose/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        # Gripper directories
        for name in self.args.gripper_encoder_names:
            dir_path = os.path.join(self.episode_dir, f"gripper/encoder/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        # IMU directories
        for name in self.args.imu_9axis_names:
            dir_path = os.path.join(self.episode_dir, f"imu/9axis/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        # Lidar directories
        for name in self.args.lidar_point_cloud_names:
            dir_path = os.path.join(self.episode_dir, f"lidar/pointCloud/{name}")
            os.makedirs(dir_path, exist_ok=True)
        
        # Robot base directories
        for name in self.args.robot_base_vel_names:
            dir_path = os.path.join(self.episode_dir, f"robotBase/vel/{name}")
            os.makedirs(dir_path, exist_ok=True)

    def init_subscribers(self):
        """Initialize all ROS subscribers"""
        # Camera color subscribers
        for i, topic in enumerate(self.args.camera_color_topics):
            rospy.Subscriber(topic, Image, self.camera_color_callback, callback_args=i)
            rospy.loginfo(f"Subscribed to camera color topic: {topic}")
        
        # Camera depth subscribers
        for i, topic in enumerate(self.args.camera_depth_topics):
            rospy.Subscriber(topic, Image, self.camera_depth_callback, callback_args=i)
        
        # Camera point cloud subscribers
        for i, topic in enumerate(self.args.camera_point_cloud_topics):
            rospy.Subscriber(topic, PointCloud2, self.camera_point_cloud_callback, callback_args=i)
        
        # Arm joint state subscribers
        for i, topic in enumerate(self.args.arm_joint_state_topics):
            rospy.Subscriber(topic, JointState, self.arm_joint_state_callback, callback_args=i)
        
        # Arm end pose subscribers
        for i, topic in enumerate(self.args.arm_end_pose_topics):
            rospy.Subscriber(topic, PoseStamped, self.arm_end_pose_callback, callback_args=i)
        
        # Localization pose subscribers
        for i, topic in enumerate(self.args.localization_pose_topics):
            rospy.Subscriber(topic, PoseStamped, self.localization_pose_callback, callback_args=i)
        
        # Gripper encoder subscribers
        if Gripper is not None:
            for i, topic in enumerate(self.args.gripper_encoder_topics):
                rospy.Subscriber(topic, Gripper, self.gripper_encoder_callback, callback_args=i)
        
        # IMU subscribers
        for i, topic in enumerate(self.args.imu_9axis_topics):
            rospy.Subscriber(topic, Imu, self.imu_9axis_callback, callback_args=i)
        
        # Lidar point cloud subscribers
        for i, topic in enumerate(self.args.lidar_point_cloud_topics):
            rospy.Subscriber(topic, PointCloud2, self.lidar_point_cloud_callback, callback_args=i)
        
        # Robot base velocity subscribers
        for i, topic in enumerate(self.args.robot_base_vel_topics):
            rospy.Subscriber(topic, Twist, self.robot_base_vel_callback, callback_args=i)

    def camera_color_callback(self, msg, index):
        """Callback for camera color images"""
        try:
            timestamp = msg.header.stamp.to_sec()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Save image
            name = self.args.camera_color_names[index]
            filename = f"{timestamp}.png"
            filepath = os.path.join(self.episode_dir, f"camera/color/{name}", filename)
            cv2.imwrite(filepath, cv_image)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"camera/color/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
            
            # Save config (intrinsics/extrinsics) on first capture
            config_file = os.path.join(self.episode_dir, f"camera/color/{name}", "config.json")
            if not os.path.exists(config_file):
                config = {
                    "width": cv_image.shape[1],
                    "height": cv_image.shape[0],
                    "K": [0.0] * 9,  # Camera intrinsics (to be filled by calibration)
                    "parent_frame": {
                        "x": 0.0, "y": 0.0, "z": 0.0,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0
                    }
                }
                with open(config_file, 'w') as f:
                    json.dump(config, f, indent=2)
                    
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error in camera_color_callback: {e}")
        except Exception as e:
            rospy.logerr(f"Error in camera_color_callback: {e}")

    def camera_depth_callback(self, msg, index):
        """Callback for camera depth images"""
        try:
            timestamp = msg.header.stamp.to_sec()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            
            # Save image
            name = self.args.camera_depth_names[index]
            filename = f"{timestamp}.png"
            filepath = os.path.join(self.episode_dir, f"camera/depth/{name}", filename)
            cv2.imwrite(filepath, cv_image)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"camera/depth/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
            
            # Save config on first capture
            config_file = os.path.join(self.episode_dir, f"camera/depth/{name}", "config.json")
            if not os.path.exists(config_file):
                config = {
                    "width": cv_image.shape[1],
                    "height": cv_image.shape[0],
                    "K": [0.0] * 9,
                    "parent_frame": {
                        "x": 0.0, "y": 0.0, "z": 0.0,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0
                    }
                }
                with open(config_file, 'w') as f:
                    json.dump(config, f, indent=2)
                    
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error in camera_depth_callback: {e}")
        except Exception as e:
            rospy.logerr(f"Error in camera_depth_callback: {e}")

    def camera_point_cloud_callback(self, msg, index):
        """Callback for camera point clouds"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Convert PointCloud2 to numpy array
            pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            
            # Save point cloud
            name = self.args.camera_point_cloud_names[index]
            filename = f"{timestamp}.npy"
            filepath = os.path.join(self.episode_dir, f"camera/pointCloud/{name}", filename)
            np.save(filepath, pc_array)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"camera/pointCloud/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
            
            # Save config on first capture
            config_file = os.path.join(self.episode_dir, f"camera/pointCloud/{name}", "config.json")
            if not os.path.exists(config_file):
                config = {
                    "K": [0.0] * 9,
                    "parent_frame": {
                        "x": 0.0, "y": 0.0, "z": 0.0,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0
                    }
                }
                with open(config_file, 'w') as f:
                    json.dump(config, f, indent=2)
                    
        except Exception as e:
            rospy.logerr(f"Error in camera_point_cloud_callback: {e}")

    def arm_joint_state_callback(self, msg, index):
        """Callback for arm joint states"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Save joint state
            name = self.args.arm_joint_state_names[index]
            filename = f"{timestamp}.json"
            filepath = os.path.join(self.episode_dir, f"arm/jointState/{name}", filename)
            
            data = {
                "position": list(msg.position),
                "velocity": list(msg.velocity) if msg.velocity else [],
                "effort": list(msg.effort) if msg.effort else []
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"arm/jointState/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in arm_joint_state_callback: {e}")

    def arm_end_pose_callback(self, msg, index):
        """Callback for arm end poses"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Save end pose
            name = self.args.arm_end_pose_names[index]
            filename = f"{timestamp}.json"
            filepath = os.path.join(self.episode_dir, f"arm/endPose/{name}", filename)
            
            # Convert quaternion to euler angles
            orientation = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(orientation)
            
            data = {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"arm/endPose/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in arm_end_pose_callback: {e}")

    def localization_pose_callback(self, msg, index):
        """Callback for localization poses"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Save localization pose
            name = self.args.localization_pose_names[index]
            filename = f"{timestamp}.json"
            filepath = os.path.join(self.episode_dir, f"localization/pose/{name}", filename)
            
            # Convert quaternion to euler angles
            orientation = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(orientation)
            
            data = {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"localization/pose/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in localization_pose_callback: {e}")

    def gripper_encoder_callback(self, msg, index):
        """Callback for gripper encoder data"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Save gripper data
            name = self.args.gripper_encoder_names[index]
            filename = f"{timestamp}.json"
            filepath = os.path.join(self.episode_dir, f"gripper/encoder/{name}", filename)
            
            data = {
                "angle": msg.angle,
                "distance": msg.distance
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"gripper/encoder/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in gripper_encoder_callback: {e}")

    def imu_9axis_callback(self, msg, index):
        """Callback for IMU data"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Save IMU data
            name = self.args.imu_9axis_names[index]
            filename = f"{timestamp}.json"
            filepath = os.path.join(self.episode_dir, f"imu/9axis/{name}", filename)
            
            data = {
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z
                }
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"imu/9axis/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in imu_9axis_callback: {e}")

    def lidar_point_cloud_callback(self, msg, index):
        """Callback for lidar point clouds"""
        try:
            timestamp = msg.header.stamp.to_sec()
            
            # Convert PointCloud2 to numpy array
            pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            
            # Save point cloud
            name = self.args.lidar_point_cloud_names[index]
            filename = f"{timestamp}.npy"
            filepath = os.path.join(self.episode_dir, f"lidar/pointCloud/{name}", filename)
            np.save(filepath, pc_array)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"lidar/pointCloud/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in lidar_point_cloud_callback: {e}")

    def robot_base_vel_callback(self, msg, index):
        """Callback for robot base velocity"""
        try:
            # Twist messages don't have headers, so use current time
            timestamp = rospy.Time.now().to_sec()
            
            # Save velocity data
            name = self.args.robot_base_vel_names[index]
            filename = f"{timestamp}.json"
            filepath = os.path.join(self.episode_dir, f"robotBase/vel/{name}", filename)
            
            data = {
                "linear_x": msg.linear.x,
                "linear_y": msg.linear.y,
                "angular_z": msg.angular.z
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f)
            
            # Save sync info
            sync_file = os.path.join(self.episode_dir, f"robotBase/vel/{name}", "sync.txt")
            with open(sync_file, 'a') as f:
                f.write(f"{filename}\n")
                
        except Exception as e:
            rospy.logerr(f"Error in robot_base_vel_callback: {e}")

    def run(self):
        """Run the data capture node"""
        rospy.spin()


def get_arguments():
    parser = argparse.ArgumentParser(description='Data capture node for Pika robot')
    parser.add_argument('--dataset_dir', type=str, default='/home/agilex/data',
                        help='Directory to save captured data')
    parser.add_argument('--episode_index', type=int, default=0,
                        help='Episode index for this capture session')
    parser.add_argument('--type', type=str, default='aloha',
                        help='Type of data capture configuration')
    parser.add_argument('--config_path', type=str, default='',
                        help='Path to configuration YAML file')
    
    args = parser.parse_args()
    
    # Override with ROS parameters if available (for launch file compatibility)
    if rospy.has_param('~dataset_dir'):
        args.dataset_dir = rospy.get_param('~dataset_dir')
    if rospy.has_param('~episode_index'):
        args.episode_index = rospy.get_param('~episode_index')
    if rospy.has_param('~type'):
        args.type = rospy.get_param('~type')
    if rospy.has_param('~config_path'):
        args.config_path = rospy.get_param('~config_path')
    
    # Load configuration from YAML
    if not args.config_path:
        # Try to find config in install directory
        config_file = f'/opt/ros/noetic/share/data_tools/config/{args.type}_data_params.yaml'
        if not os.path.exists(config_file):
            # Try relative path
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_file = os.path.join(script_dir, '..', 'config', f'{args.type}_data_params.yaml')
        args.config_path = config_file
    
    if not os.path.exists(args.config_path):
        rospy.logerr(f"Configuration file not found: {args.config_path}")
        rospy.logerr("Please ensure the config file exists or specify --config_path")
        return None
    
    with open(args.config_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
    
    # Load configuration parameters
    args.camera_color_names = yaml_data['dataInfo']['camera']['color']['names']
    args.camera_color_topics = yaml_data['dataInfo']['camera']['color']['topics']
    args.camera_depth_names = yaml_data['dataInfo']['camera']['depth']['names']
    args.camera_depth_topics = yaml_data['dataInfo']['camera']['depth']['topics']
    args.camera_point_cloud_names = yaml_data['dataInfo']['camera']['pointCloud']['names']
    args.camera_point_cloud_topics = yaml_data['dataInfo']['camera']['pointCloud']['topics']
    args.arm_joint_state_names = yaml_data['dataInfo']['arm']['jointState']['names']
    args.arm_joint_state_topics = yaml_data['dataInfo']['arm']['jointState']['topics']
    args.arm_end_pose_names = yaml_data['dataInfo']['arm']['endPose']['names']
    args.arm_end_pose_topics = yaml_data['dataInfo']['arm']['endPose']['topics']
    args.localization_pose_names = yaml_data['dataInfo']['localization']['pose']['names']
    args.localization_pose_topics = yaml_data['dataInfo']['localization']['pose']['topics']
    args.gripper_encoder_names = yaml_data['dataInfo']['gripper']['encoder']['names']
    args.gripper_encoder_topics = yaml_data['dataInfo']['gripper']['encoder']['topics']
    args.imu_9axis_names = yaml_data['dataInfo']['imu']['9axis']['names']
    args.imu_9axis_topics = yaml_data['dataInfo']['imu']['9axis']['topics']
    args.lidar_point_cloud_names = yaml_data['dataInfo']['lidar']['pointCloud']['names']
    args.lidar_point_cloud_topics = yaml_data['dataInfo']['lidar']['pointCloud']['topics']
    args.robot_base_vel_names = yaml_data['dataInfo']['robotBase']['vel']['names']
    args.robot_base_vel_topics = yaml_data['dataInfo']['robotBase']['vel']['topics']
    
    return args


def main():
    rospy.init_node('data_capture_node', anonymous=False)
    
    args = get_arguments()
    if args is None:
        return
    
    node = DataCaptureNode(args)
    node.run()


if __name__ == '__main__':
    main()
