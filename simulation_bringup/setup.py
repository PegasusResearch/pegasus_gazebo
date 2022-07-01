import os
from glob import glob
from setuptools import setup

package_name = 'simulation_bringup'

setup(
    # Add the launch files so that colcon system can discover them
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ]
)