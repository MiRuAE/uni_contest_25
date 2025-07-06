from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the parameter file
    f1tenth_stack_dir = get_package_share_directory('f1tenth_stack')
    default_param_file = os.path.join(f1tenth_stack_dir, 'config', 'vesc.yaml')
    
    # Without learning Camera
    # camera_config = os.path.join(
    #     get_package_share_directory('camera_basic_pkg'),
    #     'config',
    #     'lane_following.yaml'
    # )

    # With learning
    camera_config = os.path.join(
        get_package_share_directory('lane_mission_pkg'),
        'config',
        'lane_following.yaml'
    )

    lidar_config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'reactive_node.yaml'
    )

    odom_navigation_config = os.path.join(
        get_package_share_directory('odom_navigation'),
        'config',
        'odom_navigation.yaml'
    )

    # Declare the parameter file argument
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to the VESC parameter file'
    )

    return LaunchDescription([
        # Add the parameter file argument
        param_file_arg,

        #Node Launcher for sector detection
        # Node(
        #     package='launcher_pkg',
        #     executable='mk_node_launcher',
        #     name='mk_node_launcher',
        #     output='screen'
        # ),
        Node(
            package='rust_ws',
            executable='mission_launcher',
            name='mission_launcher',
            output='screen'
        ),
        
        # Lane Following Node for camera-based driving
        # Node(
        #     package='camera_basic_pkg',
        #     executable='lanefollowing',
        #     name='lane_following_node',
        #     output='screen',
        #     parameters=[camera_config]
        # ),

        # Lane following with learning
        Node(
            package='lane_mission_pkg',
            executable='lane_mission_node',
            name='lane_mission_node',
            output='screen',
            parameters=[camera_config]
        ),
        # Node(
        #     package='camera_basic_pkg',
        #     executable='lanefollowing_intel',
        #     name='lane_following_node',
        #     output='screen'
        # ),
        
        # Reactive Follow Gap Node for LiDAR-based driving
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            output='screen',
            parameters=[lidar_config]
        ),
        # Node(
        #     package='rust_ws',
        #     executable='mission_gap',
        #     name='mission_gap',
        #     output='screen'
        # ),
        
        # VESC to Odom Node with EKF (as a component)
        ComposableNodeContainer(
            name='vesc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='odom_publisher',
                    plugin='vesc_ackermann::VescToOdomWithEKF',
                    name='vesc_to_odom_node',
                    parameters=[LaunchConfiguration('param_file')]
                )
            ],
            output='screen'
        ),
        
        # Odom Navigation Node for position-based driving
        Node(
            package='odom_navigation',
            executable='odom_navigation_node',
            name='odom_navigation_node',
            output='screen',
            parameters=[odom_navigation_config]
        )
    ]) 
