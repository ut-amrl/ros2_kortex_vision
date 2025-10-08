from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

configurable_parameters = [
    {"name": "device", "default": "192.168.1.10", "description": "Device IPv4 address"},
    {"name": "camera", "default": "camera", "description": "'camera' should uniquely identify the device. All topics are pushed down into the 'camera' namespace."},
    {"name": "camera_link_frame_id", "default": "camera_link", "description": "Camera link frame identifier"},
    {"name": "color_frame_id", "default": "camera_color_frame", "description": "Color camera frame identifier"},
    {"name": "depth_frame_id", "default": "camera_depth_frame", "description": "Depth camera frame identifier"},
    {"name": "color_camera_info_url", "default": "", "description": "URL of custom calibration file for color camera. See camera_info_manager docs for calibration URL details"},
    {"name": "depth_camera_info_url", "default": "", "description": "URL of custom calibration file for depth camera. See camera_info_manager docs for calibration URL details"},
    {"name": "depth_rtsp_element_config", "default": "depth latency=30", "description": "RTSP element configuration for depth stream"},
    {"name": "depth_rtp_depay_element_config", "default": "rtpgstdepay", "description": "RTP element configuration for depth stream"},
    {"name": "color_rtsp_element_config", "default": "color latency=30", "description": "RTSP element configuration for color stream"},
    {"name": "color_rtp_depay_element_config", "default": "rtph264depay", "description": "RTP element configuration for color stream"},
    {"name": "launch_color", "default": "true", "description": "Launch the color image node"},
    {"name": "launch_depth", "default": "true", "description": "Launch the depth image node"},
    {"name": "depth_registration", "default": "true", "description": "Enable depth→RGB registration + masking pipeline"},
    {"name": "max_color_pub_rate", "default": "30.0", "description": "Maximum image publication rate"},
    {"name": "max_depth_pub_rate", "default": "30.0", "description": "Maximum image publication rate"},
]

def declare_configurable_parameters():
    return [
        DeclareLaunchArgument(param["name"], default_value=param["default"], description=param["description"])
        for param in configurable_parameters
    ]

def set_configurable_parameters(parameters):
    return dict([(param["name"], LaunchConfiguration(param["name"])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, *args, **kwargs):
    # Depth Node
    depth_node = Node(
        package="kinova_vision",
        namespace=LaunchConfiguration("camera"),
        executable="kinova_vision_node",
        name="kinova_vision_depth",
        output="both",
        parameters=[{
            "camera_type": "depth",
            "camera_name": "depth",
            "camera_info_url_default": "package://kinova_vision/launch/calibration/default_depth_calib_%ux%u.ini",
            "camera_info_url_user": LaunchConfiguration("depth_camera_info_url").perform(context),
            "stream_config": "rtspsrc location=rtsp://"
                + LaunchConfiguration("device").perform(context) + "/"
                + LaunchConfiguration("depth_rtsp_element_config").perform(context)
                + " ! "
                + LaunchConfiguration("depth_rtp_depay_element_config").perform(context),
            "frame_id": LaunchConfiguration("depth_frame_id").perform(context),
            "max_pub_rate": LaunchConfiguration("max_depth_pub_rate"),
        }],
        remappings=[
            ("camera_info", "depth/camera_info"),
            ("image_raw", "depth/image_raw"),
            ("image_raw/compressed", "depth/image_raw/compressed"),
            ("image_raw/compressedDepth", "depth/image_raw/compressedDepth"),
            ("image_raw/theora", "depth/image_raw/theora"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_depth")),
    )

    # Color Node
    color_node = Node(
        package="kinova_vision",
        namespace=LaunchConfiguration("camera"),
        executable="kinova_vision_node",
        name="kinova_vision_color",
        output="both",
        parameters=[{
            "camera_type": "color",
            "camera_name": "color",
            "camera_info_url_default": "package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini",
            "camera_info_url_user": LaunchConfiguration("color_camera_info_url").perform(context),
            "stream_config": "rtspsrc location=rtsp://"
                + LaunchConfiguration("device").perform(context) + "/"
                + LaunchConfiguration("color_rtsp_element_config").perform(context)
                + " ! "
                + LaunchConfiguration("color_rtp_depay_element_config").perform(context)
                + " ! avdec_h264 ! videoconvert",
            "frame_id": LaunchConfiguration("color_frame_id").perform(context),
            "max_pub_rate": LaunchConfiguration("max_color_pub_rate"),
        }],
        remappings=[
            ("camera_info", "color/camera_info"),
            ("image_raw", "color/image_raw"),
            ("image_raw/compressed", "color/image_raw/compressed"),
            ("image_raw/compressedDepth", "color/image_raw/compressedDepth"),
            ("image_raw/theora", "color/image_raw/theora"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_color")),
    )

    # Align depth → RGB using depth_image_proc::RegisterNode (no rectification step)
    depth_image_proc = ComposableNodeContainer(
        name="registered_depth_images",
        namespace=LaunchConfiguration("camera"),
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_node",
                namespace=LaunchConfiguration("camera"),
                remappings=[
                    ("rgb/camera_info", "color/camera_info"),
                    ("depth/camera_info", "depth/camera_info"),
                    ("depth/image_rect", "depth/image_raw"),  # use raw; your distortion is 0
                    # output: "depth_registered/image_rect"
                ],
                parameters=[{
                    "fill_upsampling_holes": True,   # upsample 480x270 → 1280x720 with fewer holes
                    "queue_size": 10,
                }],
            ),
            # (Drop PointCloudXyzrgbNode; not needed for your use case)
        ],
        output="both",
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    # Mask RGB where registered depth is valid; publishes /color/rgb_masked
    mask_rgb_with_depth = Node(
        package="kinova_vision",                  # <-- change to your package name
        executable="mask_rgb_with_depth.py",    # <-- ensure this script is installed/executable
        name="mask_rgb_with_depth",
        namespace=LaunchConfiguration("camera"),
        remappings=[
            ("color/image_rect_color", "color/image_raw"),                 # use your RGB raw
            ("depth_registered/image_rect", "depth_registered/image_rect"),# from RegisterNode
            ("rgb_masked/image", "color/rgb_masked"),                      # output image
        ],
        parameters=[{"depth_epsilon": 1e-6}],
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    # Relay CameraInfo so masked image has corresponding camera_info
    masked_info_relay = Node(
        package="topic_tools",
        executable="relay",
        name="rgb_masked_camera_info_relay",
        namespace=LaunchConfiguration("camera"),
        arguments=["color/camera_info", "color/rgb_masked/camera_info"],
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    # Static TFs (unchanged)
    camera_depth_tf_publisher = Node(
        package="tf2_ros",
        namespace=LaunchConfiguration("camera"),
        executable="static_transform_publisher",
        name="camera_depth_tf_publisher",
        output="both",
        arguments=[
            "-0.0195","-0.005","0","0","0","0",
            LaunchConfiguration("camera_link_frame_id"),
            LaunchConfiguration("depth_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_depth")),
    )

    camera_color_tf_publisher = Node(
        package="tf2_ros",
        namespace=LaunchConfiguration("camera"),
        executable="static_transform_publisher",
        name="camera_color_tf_publisher",
        output="both",
        arguments=[
            "0","0","0","0","0","0",
            LaunchConfiguration("camera_link_frame_id"),
            LaunchConfiguration("color_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("launch_color")),
    )

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        output="both",
        arguments=[
            "0.025","0.000","0.065", "0.0","1.5708","3.1416",
            "bracelet_link", LaunchConfiguration("camera_link_frame_id"),
        ],
    )

    return [
        depth_node,
        color_node,
        camera_depth_tf_publisher,
        camera_color_tf_publisher,
        camera_mount_tf,
        depth_image_proc,       # aligns depth→RGB
        mask_rgb_with_depth,    # publishes /color/rgb_masked
        masked_info_relay,      # publishes /color/rgb_masked/camera_info
    ]

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters() + [OpaqueFunction(function=launch_setup)]
    )
