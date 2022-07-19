import os
from glob import glob
from setuptools import setup

package_name = 'simulation_models'

setup(
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*'))
    ]
)
