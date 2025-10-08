#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

configurable_parameters = [
    {"name": "device", "default": "192.168.1.10", "description": "Device IPv4 address"},
    {"name": "camera", "default": "camera", "description": "Unique namespace for the camera"},
    {"name": "camera_link_frame_id", "default": "wrist_camera_link", "description": "Camera link frame identifier"},
    {"name": "color_frame_id", "default": "camera_color_frame", "description": "Color camera frame identifier"},
    {"name": "depth_frame_id", "default": "camera_depth_frame", "description": "Depth camera frame identifier"},
    {"name": "color_optical_frame_id", "default": "camera_color_optical_frame", "description": "Color optical frame"},
    {"name": "depth_optical_frame_id", "default": "camera_depth_optical_frame", "description": "Depth optical frame"},
    {"name": "color_camera_info_url", "default": "", "description": "Custom calibration file URL for color camera"},
    {"name": "depth_camera_info_url", "default": "", "description": "Custom calibration file URL for depth camera"},
    {"name": "depth_rtsp_element_config", "default": "depth latency=30", "description": "RTSP config for depth"},
    {"name": "depth_rtp_depay_element_config", "default": "rtpgstdepay", "description": "RTP config for depth"},
    {"name": "color_rtsp_element_config", "default": "color latency=30", "description": "RTSP config for color"},
    {"name": "color_rtp_depay_element_config", "default": "rtph264depay", "description": "RTP config for color"},
    {"name": "launch_color", "default": "true", "description": "Launch the color stream"},
    {"name": "launch_depth", "default": "true", "description": "Launch the depth stream"},
    {"name": "depth_registration", "default": "true", "description": "Enable depth→RGB registration + masking"},
    {"name": "max_color_pub_rate", "default": "30.0", "description": "Max pub rate for color"},
    {"name": "max_depth_pub_rate", "default": "30.0", "description": "Max pub rate for depth"},
]

def declare_configurable_parameters():
    return [DeclareLaunchArgument(param["name"], default_value=param["default"], description=param["description"])
            for param in configurable_parameters]

def set_configurable_parameters(parameters):
    return dict([(param["name"], LaunchConfiguration(param["name"])) for param in parameters])

def launch_setup(context, *args, **kwargs):
    # --------------------------------------------------------------------------
    # Color Node
    # --------------------------------------------------------------------------
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
                + LaunchConfiguration("device").perform(context)
                + "/"
                + LaunchConfiguration("color_rtsp_element_config").perform(context)
                + " ! "
                + LaunchConfiguration("color_rtp_depay_element_config").perform(context)
                + " ! avdec_h264 ! videoconvert",
            "frame_id": LaunchConfiguration("color_optical_frame_id").perform(context),
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

    # --------------------------------------------------------------------------
    # Depth Node
    # --------------------------------------------------------------------------
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
                + LaunchConfiguration("device").perform(context)
                + "/"
                + LaunchConfiguration("depth_rtsp_element_config").perform(context)
                + " ! "
                + LaunchConfiguration("depth_rtp_depay_element_config").perform(context),
            "frame_id": LaunchConfiguration("depth_optical_frame_id").perform(context),
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

    # --------------------------------------------------------------------------
    # Depth → RGB registration
    # --------------------------------------------------------------------------
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
                    ("depth/image_rect", "depth/image_raw"),
                ],
                parameters=[{
                    "fill_upsampling_holes": True,
                    "queue_size": 10,
                }],
            ),
        ],
        output="both",
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    # --------------------------------------------------------------------------
    # RGB masking
    # --------------------------------------------------------------------------
    mask_rgb_with_depth = Node(
        package="kinova_vision",
        executable="mask_rgb_with_depth.py",
        name="mask_rgb_with_depth",
        namespace=LaunchConfiguration("camera"),
        parameters=[{
            "rgb_topic": "color/image_raw",
            "depth_topic": "depth/image_raw",
            "depth_info_topic": "depth/camera_info",
            "out_image_topic": "color/rgb_masked",
            "out_info_topic": "color/rgb_masked/camera_info",
            "min_depth_m": 0.10,
            "max_depth_m": 3.5,
            "sync_queue": 10,
            "sync_slop": 0.05
        }],
    )

    masked_info_relay = Node(
        package="topic_tools",
        executable="relay",
        name="rgb_masked_camera_info_relay",
        namespace=LaunchConfiguration("camera"),
        arguments=["color/camera_info", "color/rgb_masked/camera_info"],
        condition=IfCondition(LaunchConfiguration("depth_registration")),
    )

    # --------------------------------------------------------------------------
    # Static TFs (5 total)
    # --------------------------------------------------------------------------
    bracelet_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bracelet_to_camera_link",
        output="both",
        arguments=[
            "0", "-0.05", "-0.07", "0", "0", "0",
            "bracelet_link", LaunchConfiguration("camera_link_frame_id"),
        ],
    )

    camera_to_color_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_color_frame",
        output="both",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            LaunchConfiguration("camera_link_frame_id"), LaunchConfiguration("color_frame_id"),
        ],
    )

    camera_to_depth_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_depth_frame",
        output="both",
        arguments=[
            "-0.0195", "-0.005", "0", "0", "0", "0",
            LaunchConfiguration("camera_link_frame_id"), LaunchConfiguration("depth_frame_id"),
        ],
    )

    color_to_color_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="color_to_color_optical",
        output="both",
        arguments=[
            "0", "0", "0", "-3.1415", "0", "-3.1415",
            LaunchConfiguration("color_frame_id"), LaunchConfiguration("color_optical_frame_id"),
        ],
    )

    depth_to_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_to_depth_optical",
        output="both",
        arguments=[
            "0", "0", "0", "-3.1415", "0", "-3.1415",
            LaunchConfiguration("depth_frame_id"), LaunchConfiguration("depth_optical_frame_id"),
        ],
    )

    return [
        depth_node,
        color_node,
        depth_image_proc,
        mask_rgb_with_depth,
        masked_info_relay,
        bracelet_to_camera_link,
        camera_to_color_frame,
        camera_to_depth_frame,
        color_to_color_optical,
        depth_to_depth_optical,
    ]

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters() + [OpaqueFunction(function=launch_setup)]
    )
