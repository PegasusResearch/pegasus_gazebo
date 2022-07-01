#!/usr/bin/env python3
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # ---------------------------
    # Setup environment variables
    # ---------------------------
    
    # Get the PX4 install directory
    px4_dir = os.environ['PX4_DIR']
    px4_run_dir = os.environ['HOME'] + '/tmp/px4_run_dir'
    
    # Set the Environment variables to use the PX4 modules as a part of ROS2
    SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', px4_dir + '/build/px4_sitl_default/build_gazebo'),
    SetEnvironmentVariable('GAZEBO_MODEL_PATH', px4_dir + '/Tools/sitl_gazebo/models'),
    SetEnvironmentVariable('PX4_SIM_MODEL', 'blackdrone_gimbal')
    
    # Define a PX4 running directory for temporary files
    os.makedirs(os.environ['HOME'] + '/tmp/px4_run_dir', exist_ok=True)
    
    # -----------------------------------
    # Setup the gazebo server and client
    # -----------------------------------
    
    # 1. Get the location of the ros_gazebo package
    gazebo_package_location = get_package_share_directory('gazebo_ros')
    
    # 2. Launch the gazebo server simulator with a specific world
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_package_location, '/launch/gzserver.launch.py']),
        launch_arguments={
            'world': 'worlds/empty.world', 
            'verbose': 'true'}.items(),
    )
    
    # 3. Launch the gazebo graphical client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_package_location, '/launch/gzclient.launch.py'])
    )
    
    mavlink_sitl_gazebo = os.path.join(px4_dir, 'Tools/sitl_gazebo')
    
    # 4. Generate the SDF vehicle model by replacing in the jinja file, the communication scheme for mavlink
    sdf_model_generator = subprocess.run(
        os.path.join(mavlink_sitl_gazebo, 'scripts/jinja_gen.py') +
        ' --stdout --mavlink_id=1 --mavlink_udp_port=14560 --mavlink_tcp_port=4560 --gst_udp_port=5600 --video_uri=5600 --mavlink_cam_udp_port=14530 ' + 
        ' --output-file=models/iris/iris.sdf ' +
        os.path.join(mavlink_sitl_gazebo, 'models/iris/iris.sdf.jinja') + ' ' + mavlink_sitl_gazebo, 
        stdout=subprocess.PIPE, 
        stderr=subprocess.PIPE,
        text=True,
        shell=True)
    print(sdf_model_generator.stderr)
    
    model = os.path.join(px4_dir, 'Tools', 'sitl_gazebo', 'models', 'iris', 'iris.sdf')
    
    # 5. Spawn the SDF gazebo model
    spawn_gazebo_vehicle_node = ExecuteProcess(
        cmd=[
            'gz', 'model',
            '--spawn-file', model,
            '--model-name', 'drone',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        #prefix="bash -c 'sleep 30s; $0 $@'",
        output='screen'
    )
    
    # "/Users/marcelojacinto/ros2/PX4-Autopilot/build/px4_sitl_default/bin/px4" "/Users/marcelojacinto/ros2/PX4-Autopilot/build/px4_sitl_default"/etc -s etc/init.d-posix/rcS -t "/Users/marcelojacinto/ros2/PX4-Autopilot"/test_data
    
    # 4. Launch the PX4
    # px4_sitl_sim = ExecuteProcess(
    #     cmd=[px4_dir + '/build/px4_sitl_default/bin/px4',
    #          '-s init.d-posix/rcS',
    #          ''],
    #     cwd=px4_dir + '/build/px4_sitl_default/etc',
    #     output='screen')
    
    # 
    #
    # ./px4 /Users/marcelojacinto/ros2/PX4-Autopilot/build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i 0 -w sitl_iris_0 -d
    
    # ----------------------------------------
    # ---- DECLARE THE LAUNCH ARGUMENTS ------
    # ----------------------------------------

    # Namespace and ID of the vehicle as parameter received by the launch file
    id_arg = DeclareLaunchArgument('vehicle_id', default_value='0', description='Drone ID in the network')
    namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone', description='Namespace to append to every topic and node name')
    
    mavlink_id_arg = DeclareLaunchArgument('mavlink_id', default_value='0', description='Mavlink ID of the vehicle')
    
    # Define the drone MAVLINK IP and PORT
    mav_connection_arg = DeclareLaunchArgument('connection', default_value='udp://:14550', description='The interface used to connect to the vehicle')
    # Other possible connection values
    #connection: "udp://:15006"    # snapdragon 6
    #connection: "udp://:15000"
    #connection: "serial:///dev/ttyACM0:57600"
    
    # ----------------------------------------
    # ---- DECLARE THE NODES TO LAUNCH -------
    # ----------------------------------------
    
    # 1. Get the PX4 simulator install directory
    
    # mavlink_sitl_gazebo = os.path.join(px4_dir, 'Tools/sitl_gazebo')
    
    #sdf_model = sdf_model_generator.stdout
    # print(sdf_model_generator.stdout)
    
    # 3. Start the PX4 SITL simulation (simulate the Pixhawk microcontroller running PX4)
    # px4_sim = ExecuteProcess(
    #     cmd=[
    #         os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'),
    #         os.path.join(px4_dir, 'build/px4_sitl_default/etc'),
    #         '-s etc/init.d-posix/rcS -i',
    #         LaunchConfiguration('vehicle_id'),
    #         '-w sdf_iris1'],
    #     output='screen',
    #     emulate_tty=True
    # )
    
    # print(px4_sim.output)
    
    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        # Launch arguments
        id_arg, 
        namespace_arg,
        mavlink_id_arg,
        # Launch files
        gazebo_server,
        gazebo_client,
        spawn_gazebo_vehicle_node
    ])