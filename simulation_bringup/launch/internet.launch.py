"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    HOME = os.environ.get('HOME')
    PX4_RUN_DIR = HOME + '/tmp/px4_run_dir'
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    fpv_racing_gazebo_dir = '/Users/marcelojacinto/ros2/PX4-Autopilot/Tools/sitl_gazebo'
    world = os.path.join(gazebo_launch_dir, 'worlds', 'empty.world')
    model = os.path.join(fpv_racing_gazebo_dir, 'models', 'iris', 'iris.sdf')
    #custom_gazebo_models = os.path.join(blackdrones_description_dir, 'models')
    #px4_init = os.path.join(blackdrones_description_dir, 'PX4-init')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', HOME + '/ros2/PX4-Autopilot/build/px4_sitl_default/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', HOME + '/ros2/PX4-Autopilot/Tools/sitl_gazebo/models'),
        #SetEnvironmentVariable('LD_LIBRARY_PATH', '')

        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),

        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
        #     launch_arguments={'world': LaunchConfiguration('world'),
        #                       'verbose': 'true'}.items(),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        # ),
        
       
        # Node(package='gazebo_ros', executable='spawn_entity.py',
        #     arguments=[
        #         '-entity', 
        #         'demo', 
        #         '-database', 
        #         'double_pendulum_with_base'],
        #     output='screen'),

        ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', LaunchConfiguration('model'),
                '--model-name', 'drone',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y')
            ],
            #prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),
            
        ExecuteProcess(
            cmd=[
                HOME + '/ros2/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                HOME + '/ros2/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/ros2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS'
            ],
            prefix='bash -c "sleep 5; $0 $@"',
            cwd=PX4_RUN_DIR,
            output='screen'),
])