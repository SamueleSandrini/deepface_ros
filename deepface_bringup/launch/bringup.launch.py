from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    #
    # ARGS
    #
    # model = LaunchConfiguration("model")
    # model_cmd = DeclareLaunchArgument(
    #     "model",
    #     default_value="yolov8m.pt",
    #     description="Model name or path")

    # tracker = LaunchConfiguration("tracker")
    # tracker_cmd = DeclareLaunchArgument(
    #     "tracker",
    #     default_value="bytetrack.yaml",
    #     description="Tracker name or path")

    # device = LaunchConfiguration("device")
    # device_cmd = DeclareLaunchArgument(
    #     "device",
    #     default_value="cuda:0",
    #     description="Device to use (GPU/CPU)")

    # enable = LaunchConfiguration("enable")
    # enable_cmd = DeclareLaunchArgument(
    #     "enable",
    #     default_value="True",
    #     description="Whether to start YOLOv8 enabled")

    # threshold = LaunchConfiguration("threshold")
    # threshold_cmd = DeclareLaunchArgument(
    #     "threshold",
    #     default_value="0.5",
    #     description="Minimum probability of a detection to be published")

    # input_image_topic = LaunchConfiguration("input_image_topic")
    # input_image_topic_cmd = DeclareLaunchArgument(
    #     "input_image_topic",
    #     default_value="/camera/rgb/image_raw",
    #     description="Name of the input image topic")

    # image_reliability = LaunchConfiguration("image_reliability")
    # image_reliability_cmd = DeclareLaunchArgument(
    #     "image_reliability",
    #     default_value="2",
    #     choices=["0", "1", "2"],
    #     description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")

    # input_depth_topic = LaunchConfiguration("input_depth_topic")
    # input_depth_topic_cmd = DeclareLaunchArgument(
    #     "input_depth_topic",
    #     default_value="/camera/depth/image_raw",
    #     description="Name of the input depth topic")

    # depth_image_reliability = LaunchConfiguration("depth_image_reliability")
    # depth_image_reliability_cmd = DeclareLaunchArgument(
    #     "depth_image_reliability",
    #     default_value="2",
    #     choices=["0", "1", "2"],
    #     description="Specific reliability QoS of the input depth image topic (0=system default, 1=Reliable, 2=Best Effort)")

    # input_depth_info_topic = LaunchConfiguration("input_depth_info_topic")
    # input_depth_info_topic_cmd = DeclareLaunchArgument(
    #     "input_depth_info_topic",
    #     default_value="/camera/depth/camera_info",
    #     description="Name of the input depth info topic")

    # depth_info_reliability = LaunchConfiguration("depth_info_reliability")
    # depth_info_reliability_cmd = DeclareLaunchArgument(
    #     "depth_info_reliability",
    #     default_value="2",
    #     choices=["0", "1", "2"],
    #     description="Specific reliability QoS of the input depth info topic (0=system default, 1=Reliable, 2=Best Effort)")

    # depth_image_units_divisor = LaunchConfiguration(
    #     "depth_image_units_divisor")
    # depth_image_units_divisor_cmd = DeclareLaunchArgument(
    #     "depth_image_units_divisor",
    #     default_value="1000",
    #     description="Divisor used to convert the raw depth image values into metres")

    # target_frame = LaunchConfiguration("target_frame")
    # target_frame_cmd = DeclareLaunchArgument(
    #     "target_frame",
    #     default_value="base_link",
    #     description="Target frame to transform the 3D boxes")

    # maximum_detection_threshold = LaunchConfiguration(
    #     "maximum_detection_threshold")
    # maximum_detection_threshold_cmd = DeclareLaunchArgument(
    #     "maximum_detection_threshold",
    #     default_value="0.3",
    #     description="Maximum detection threshold in the z axis")

    # namespace = LaunchConfiguration("namespace")
    # namespace_cmd = DeclareLaunchArgument(
    #     "namespace",
    #     default_value="yolo",
    #     description="Namespace for the nodes")

    #
    # NODES
    #
    deepface_cmd = Node(
        package="deepface_ros",
        executable="deepface_node",
        name="deepface_node"
    )


    ld = LaunchDescription()

    ld.add_action(deepface_cmd)

    return ld