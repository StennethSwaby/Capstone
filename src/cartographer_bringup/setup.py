from setuptools import setup
from glob import glob
import os

package_name = 'cartographer_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # optional but recommended if you have a src/ dir
    data_files=[
        # Required by ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch and config files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Add your missing resource folders
        (os.path.join('share', package_name, 'description'),
         glob('description/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'test'), glob('test/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stenneth Swaby',
    maintainer_email='swabys1@tcnj.edu',
    description='Cartographer bringup (2D) for sim/robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Add your Python executables here if you have any
            # e.g. 'my_node = cartographer_bringup.my_node:main',
        ],
    },
)
