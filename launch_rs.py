from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',   # ← 关键改正
            namespace='gripper',
            name='rsCamera',
            
            parameters=[{
                'device_type': 'd435',
                'serial_no': '135122075071',

                'camera_namespace': 'gripper',
                'camera_name': 'rsCamera',
                # ===== 1. 不开启点云 =====
                'pointcloud.enable': False,

                # ===== 2. 640x480@30 Hz =====
                'rgb_camera.color_profile': '640,480,30',

                # 其余沿用你原来的默认值
                'enable_color': True,
                'enable_depth': False,
                'align_depth.enable': False,
                # 其他参数按需继续加
            }]
        )
    ])