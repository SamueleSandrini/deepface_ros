
# deepface_ros

ROS 2 wrap for [DeepFace](https://github.com/serengil/deepface) to perform face analysis.

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/SamueleSandrini/deepface_ros
pip3 install -r deepface_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Usage

Launch the deepface_ros_node using the launcher filer:

```shell
ros2 launch deepface_bringup bringup.launch.py
```

## Subscribers

- **/image** ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)): Subscribes to raw image data for face analysis.

## Publishers

- **/faces_analysis** ([deepface_msgs/FacesAnalysis](#)): Publishes analyzed faces data including emotions, gender, race, and more.

## Services

- **/faces_analysis** ([deepface_msgs/FacesAnalysis](#)): Service for analyzing faces in the last acquired image.
- **/image_faces_analysis** ([deepface_msgs/ImageFacesAnalysis](#)): Service for analyzing faces in a provided image.

## Parameters

- **publish_online_analysis** (bool, default: True): Enables online publishing of face analysis results.
- **image_reliability** (int, default: 1): Reliability policy for image subscriptions.
