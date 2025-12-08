# Data Tools Package

This package provides data capture tools for the Pika robot system, including support for capturing image data from various camera sources.

## Features

- Real-time data capture from multiple sensor sources
- Support for camera color images (including image_top interface)
- Support for depth images and point clouds
- Joint states, end effector poses, and gripper data capture
- IMU and localization data capture
- Configurable data capture pipeline

## Image Top Interface

The package includes support for an `image_top` camera interface that subscribes to ROS Image messages (which can originate from OpenCV camera nodes or other image sources) and saves them alongside other sensor data. This interface is configured in the data capture configuration file.

## Usage

### Launch Data Capture

For ROS1 (noetic):
```bash
roslaunch data_tools run_data_capture.launch dataset_dir:=/path/to/data episode_index:=0
```

For ROS2:
```bash
ros2 launch data_tools run_data_capture.launch.py dataset_dir:=/path/to/data episode_index:=0
```

### Configuration

The data capture configuration is defined in `config/aloha_data_params.yaml`. The image_top camera is configured as follows:

```yaml
dataInfo:
  camera:
    color:
      names: ['left', 'right', 'front', 'image_top']
      topics: ['/camera/color/left/image_raw', '/camera/color/right/image_raw', '/camera/color/front/image_raw', '/camera/color/image_top/image_raw']
```

To capture data from the image_top camera, ensure your camera node publishes to the `/camera/color/image_top/image_raw` topic.

### Data Structure

Captured data is organized in the following structure:

```
dataset_dir/
  episode0/
    camera/
      color/
        left/
        right/
        front/
        image_top/         # Image data from the top camera
          *.png            # Image files
          sync.txt         # Timestamp synchronization
          config.json      # Camera configuration
      depth/
      pointCloud/
    arm/
      jointState/
      endPose/
    gripper/
    imu/
    localization/
```

### Converting to HDF5

After capturing data, convert it to HDF5 format using:

```bash
python scripts/data_to_hdf5.py --datasetDir /path/to/data --episodeIndex 0 --type aloha
```

## Dependencies

- ROS (noetic or higher)
- cv_bridge
- sensor_msgs
- geometry_msgs
- data_msgs
- Python packages: numpy, opencv-python, yaml, ros_numpy

## Topics

The data capture node subscribes to the following topics (configurable via YAML):

### Camera Topics
- `/camera/color/*/image_raw` - Color images
- `/camera/depth/*/image_raw` - Depth images  
- `/camera/pointCloud/*` - Point clouds

### Robot Topics
- `/arm/jointState/*` - Joint states
- `/arm/endPose/*` - End effector poses
- `/gripper/encoder/*` - Gripper encoder data
- `/localization/pose/*` - Localization poses
- `/imu/9axis/*` - IMU data

## Notes

- The image_top interface accepts standard ROS Image messages
- Images are saved as PNG files with timestamps
- Configuration includes camera intrinsics and extrinsics (to be calibrated)
- All data is timestamped and synchronized via sync.txt files
