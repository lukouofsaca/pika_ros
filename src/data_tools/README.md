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

### Example: Publishing from OpenCV Camera

An example script is provided to demonstrate publishing images from an OpenCV camera to the image_top topic:

```bash
rosrun data_tools example_image_publisher.py
```

This script captures images from the default camera (device 0) and publishes them to `/camera/color/image_top/image_raw` at 30 Hz.

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

## Complete Usage Example

Here's a complete workflow for capturing data with the image_top camera:

1. **Start the example image publisher** (in terminal 1):
   ```bash
   rosrun data_tools example_image_publisher.py
   ```

2. **Start the data capture node** (in terminal 2):
   ```bash
   roslaunch data_tools run_data_capture.launch dataset_dir:=/home/agilex/data episode_index:=0
   ```

3. **Perform your robotic task** while the data is being captured.

4. **Stop both nodes** when done (Ctrl+C).

5. **Convert captured data to HDF5**:
   ```bash
   cd /home/runner/work/pika_ros/pika_ros/scripts
   python data_to_hdf5.py --datasetDir /home/agilex/data --episodeIndex 0 --type aloha
   ```

The captured data will be saved in `/home/agilex/data/episode0/camera/color/image_top/` and included in the generated HDF5 file.

## Notes

- The image_top interface accepts standard ROS Image messages
- Images are saved as PNG files with timestamps
- Configuration includes camera intrinsics and extrinsics (to be calibrated)
- All data is timestamped and synchronized via sync.txt files
- The image_top camera can come from any source (USB camera, network camera, simulation, etc.) as long as it publishes ROS Image messages
