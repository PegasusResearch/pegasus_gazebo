#!/usr/bin/env python3
import os
import sys
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, LogInfo, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LocalSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    
    # Define the model to launch
    vehicle_model = 'pegasus_iris_fpv'
    
    # --------------------------------
    # PX4 and Gazebo model directories
    # --------------------------------
    
    # Get the home and px4 directories and define where temporary files are placed
    PX4_DIR = os.environ.get('PX4_DIR')
    PX4_RUN_DIR = os.environ.get('HOME') + '/tmp/px4_run_dir'
    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    # Get the PX4-gazebo directory
    px4_gazebo_dir = os.path.join(PX4_DIR, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic')
    pegasus_models_dir = get_package_share_directory('pegasus_gazebo')
    
    # Get the standard iris drone models inside the PX4 package
    model = os.path.join(pegasus_models_dir, 'models', vehicle_model, vehicle_model + '.sdf')
    
    # --------------------------------
    # Define the vehicle ID
    # --------------------------------
    
    # Set the default vehicle id (note: this is a trick due to the parameter reading limitation in ROS2)
    default_vehicle_id = 1
    vehicle_id = default_vehicle_id
    for arg in sys.argv:
        if arg.startswith('vehicle_id:='):
            vehicle_id = int(arg.split(':=')[1])
    port_increment = vehicle_id - 1
            
    # ---------------------------------------------------------------------
    # Create the Processes that need to be launched to simulate the vehicle
    # ---------------------------------------------------------------------
    
    # Note, we first need to generate the camera sdf with the correct parameters, in order to specify the robot namespace for that component of the robot
    fpv_camera_generator_proccess = ExecuteProcess(
        cmd=[
            os.path.join(get_package_prefix('pegasus_gazebo'), 'lib/pegasus_gazebo/jinja_gen.py'),
            os.path.join(pegasus_models_dir, 'models/pegasus_fpv_cam/pegasus_fpv_cam.sdf.jinja'),
            pegasus_models_dir,
            '--robot_namespace=' + 'drone' + str(vehicle_id),
            '--output-file=' + os.path.join(pegasus_models_dir, 'models/pegasus_fpv_cam/pegasus_fpv_cam.sdf'),
        ],
        output='screen',
    )
    
    # Generate the 3D model by replacing in the mavlink configuration parameters, according to the vehicle ID
    # for the IRIS robot
    model_generator_process = ExecuteProcess(
        cmd=[
            os.path.join(get_package_prefix('pegasus_gazebo'), 'lib/pegasus_gazebo/jinja_gen.py'),
            os.path.join(pegasus_models_dir, 'models/' + vehicle_model + '/' + vehicle_model + '.sdf.jinja'),
            pegasus_models_dir,
            '--mavlink_id=' + str(vehicle_id),
            '--mavlink_udp_port=' + str(14540 + port_increment),
            '--mavlink_tcp_port=' + str(4560 + port_increment),
            '--gst_udp_port=' + str(5600 + port_increment),
            '--video_uri=' + str(5600 + port_increment),
            '--mavlink_cam_udp_port=' + str(14530 + port_increment),
            '--output-file=' + os.path.join(pegasus_models_dir, 'models/'+ vehicle_model + '/' + vehicle_model + '.sdf'),
        ],
        output='screen',
    )
    
    # Spawn the 3D model in the gazebo world (it requires that a gzserver is already running)
    spawn_3d_model = ExecuteProcess(
        cmd=[
            'gz', 'model',
            '--spawn-file', model,
            '--model-name', 'drone' + str(vehicle_id),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('R'),
            '-P', LaunchConfiguration('P'),
            '-Y', LaunchConfiguration('Y')
        ],
        output='screen')
            
    # Launch PX4 simulator
    px4_sitl_process = ExecuteProcess(
        cmd=[
            PX4_DIR + '/build/px4_sitl_default/bin/px4',
            PX4_DIR + '/ROMFS/px4fmu_common/',
            '-s',
            PX4_DIR + '/ROMFS/px4fmu_common/init.d-posix/rcS',
            '-i ' + str(port_increment)
        ],
        prefix='bash -c "$0 $@"',
        cwd=PX4_RUN_DIR,
        output='screen'
    )
    
    # Launch the pegasus control and navigation code stack
    pegasus_launch = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pegasus'), 'launch/simulation/iris.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'id': str(vehicle_id), 
            'namespace': 'drone',
            'connection': 'udp://:' + str(14540 + port_increment)
        }.items()
    )

    return LaunchDescription([
        
        # Define the environment variables so that gazebo can discover PX4 3D models and plugins
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', PX4_DIR + '/build/px4_sitl_default/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', PX4_DIR + '/Tools/sitl_gazebo/models' + ':' + get_package_share_directory('pegasus_gazebo') + '/models'),
        SetEnvironmentVariable('PX4_SIM_MODEL', 'iris'),

        # Define where to spawn the vehicle (in the inertial frame) 
        # TODO - receive coordinates in ned perform the conversion to ENU and f.l.u here
        # so that the user only needs to work in NED coordinates
        DeclareLaunchArgument('vehicle_id', default_value=str(default_vehicle_id), description='Drone ID in the network'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position expressed in ENU'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position expressed in ENU'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z position expressed in ENU'),
        DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
        DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),
        
        # Declare the camera generator and the drone generate_model_process
        fpv_camera_generator_proccess,
        
        # After the sdf model generator finishes, then launch the vehicle spawn in gazebo
        RegisterEventHandler(
            OnProcessExit(
                target_action=fpv_camera_generator_proccess,
                on_exit=[
                    LogInfo(msg='Camera SDF model generated'),
                    model_generator_process
                ]
            )
        ),
        
        # After the sdf model generator finishes, then launch the vehicle spawn in gazebo
        RegisterEventHandler(
            OnProcessExit(
                target_action=model_generator_process,
                on_exit=[
                    LogInfo(msg='Vehicle SDF model generated'),
                    spawn_3d_model
                ]
            )
        ),
        
        # After the sdf model of the vehicle get's spawn on the vehicle, execute the PX4 simulator
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_3d_model,
                on_exit=[
                    LogInfo(msg='Vehicle spawned in gazebo'),
                    px4_sitl_process
                ]
            )
        ),
        
        # Launch the pegasus file that is used to spawn the pegasus control and navigation code stack
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_3d_model,
                on_exit=[
                    LogInfo(msg='Launching the Pegasus code stack!'),
                    pegasus_launch
                ]
            )
        )
])