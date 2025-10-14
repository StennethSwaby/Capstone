from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'slam_toolbox_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        # Register the package with ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Include package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch, config, and rviz directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stenneth Swaby',
    maintainer_email='swabys1@tcnj.edu',
    description='Bringup package for running SLAM Toolbox with TurtleBot3 in Gazebo and RViz.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
