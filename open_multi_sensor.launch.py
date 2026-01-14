import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_tools')

    declared_arguments = [
        DeclareLaunchArgument('name', default_value=''),
        DeclareLaunchArgument('name_index', default_value=''),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_profile', default_value='640x480x30'),
        DeclareLaunchArgument('l_fisheye_port', default_value='22'),
        DeclareLaunchArgument('r_fisheye_port', default_value='23'),
        DeclareLaunchArgument('l_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('r_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('l_depth_camera_no', default_value='_230322270688'),
        DeclareLaunchArgument('r_depth_camera_no', default_value='_230322272619'),
        DeclareLaunchArgument('l_joint_name', default_value='gripper_l_center_joint'),
        DeclareLaunchArgument('r_joint_name', default_value='gripper_r_center_joint')
    ]
    name = LaunchConfiguration('name')
    name_index = LaunchConfiguration('name_index')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_height = LaunchConfiguration('camera_height')
    camera_width = LaunchConfiguration('camera_width')
    camera_profile = LaunchConfiguration('camera_profile')
    l_fisheye_port = LaunchConfiguration('l_fisheye_port')
    r_fisheye_port = LaunchConfiguration('r_fisheye_port')
    l_serial_port = LaunchConfiguration('l_serial_port')
    r_serial_port = LaunchConfiguration('r_serial_port')
    l_depth_camera_no = LaunchConfiguration('l_depth_camera_no')
    r_depth_camera_no = LaunchConfiguration('r_depth_camera_no')
    l_joint_name = LaunchConfiguration('l_joint_name')
    r_joint_name = LaunchConfiguration('r_joint_name')

    locator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('pika_locator'), 'launch', 'pika_double_locator.launch.py')])
    )

    l_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': l_depth_camera_no,
                          'camera_namespace': name,
                          'camera_name': "camera_l",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile,
                          

                          'enable_rgbd': 'false',
                          'enable_depth': 'false',
                          'pointcloud.enable': 'false',
                          'align_depth.enable': 'false',
                          'enable_sync': 'true',                          
                          }.items()
    )
    r_depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={'serial_no': r_depth_camera_no,
                          'camera_namespace': name,
                          'camera_name': "camera_r",
                          'rgb_camera.color_profile': camera_profile, 
                          'depth_module.color_profile': camera_profile, 
                          'depth_module.depth_profile': camera_profile,
                          'depth_module.infra_profile': camera_profile,
                          

                          'enable_rgbd': 'false',
                          'enable_depth': 'false',
                          'pointcloud.enable': 'false',
                          'align_depth.enable': 'false',
                          'enable_sync': 'true',                          
                          }.items()
    )

    return LaunchDescription(declared_arguments+[
        locator_launch,
        l_depth_camera_launch,
        r_depth_camera_launch,
    ])
