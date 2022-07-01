"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = HOME + '/tmp/px4_run_dir'
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    fpv_racing_gazebo_dir = '/Users/marcelojacinto/ros2/PX4-Autopilot/Tools/sitl_gazebo'
    world = 'empty.world' #os.path.join(gazebo_launch_dir, 'worlds', 'empty.world')
   

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', HOME + '/ros2/PX4-Autopilot/build/px4_sitl_default/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/ros2/PX4-Autopilot/Tools/sitl_gazebo/models'),

        DeclareLaunchArgument('world', default_value=world),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'verbose': 'true'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py']))
       
])