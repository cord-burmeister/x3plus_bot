from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='rs',


            # The RealSense wrapper internally expects its parameters under the node name, not the namespace.
            # And the wrapper is known to behave inconsistently when the node name and namespace are identical.
            # This is a known quirk of the RealSense ROS2 wrapper.
            namespace='camera',

            output='screen',
            parameters=[
                # Core streams
                {'enable_depth': True},
                {'enable_color': True},

                # Align depth to RGB
                {'align_depth': True},


                # Point cloud
                {'pointcloud.enable': True},  # We disable the pointcloud generation in the wrapper
                # because we generate it ourselves in a separate node.
                {'pointcloud.stream_filter': 2},          # 2 = color stream
                {'pointcloud.stream_format_filter': 5},   # 5 = RGB8

                # Optional tuning
                {'depth_module.profile': '640x480x30'},
                {'rgb_camera.profile': '640x480x30'},
                {'clip_distance': 5.0},                  # meters

                # Other settings can be added here
                # The RealSense SDK (librealsense) supports these values:
                # Value	Meaning
                # 0	Disabled
                # 1	50 Hz anti‑flicker
                # 2	60 Hz anti‑flicker
                # 3	Auto (SDK only, not supported in ROS wrapper !!!!)
                # The ROS wrapper exposes only the integer range [0, 2]
                {'rgb_camera.power_line_frequency': 1},  # For Germany, 50 Hz is correct
                
            ]
        )
    ])
