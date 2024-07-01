from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():


    # model = LaunchConfiguration("model")
    # model_cmd = DeclareLaunchArgument(
    #     "model",
    #     default_value="VGG-Face",
    #     description="Model name or path")


    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the nodes")

    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")

    publish_online_analysis = LaunchConfiguration("publish_online_analysis")
    publish_online_analysis_cmd = DeclareLaunchArgument(
        "publish_online_analysis",
        default_value="true",
        description="Publish online analysis results") 
    
    #
    # NODES
    #
    deepface_cmd = Node(
        package="deepface_ros",
        executable="deepface_node",
        name="deepface_node",
        namespace=namespace,
        parameters=[{
            # "model": model,
            "publish_online_analysis": publish_online_analysis,
            "image_reliability": image_reliability,
        }],
        remappings=[
            ("image", "/head_front_camera/rgb/image_raw"),
        ]
    )


    ld = LaunchDescription()

    # Args
    ld.add_action(namespace_cmd)
    ld.add_action(image_reliability_cmd)
    ld.add_action(publish_online_analysis_cmd)

    # Nodes
    ld.add_action(deepface_cmd)

    return ld