from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'origin_frame':'map',
          'target_frame':'arm_link_fngr',
          'csv_file_name':'/home/spot_ws/path.csv'}
          ]

    return LaunchDescription([


        Node(
            package='misc_tools', executable='traj_publisher', output='screen',
            parameters=parameters,
            arguments=[]),

    ])