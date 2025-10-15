#!/usr/bin/env python3
from setuptools import setup, find_packages
from glob import glob
from pathlib import Path
import os

package_name = 'slam_toolbox_bringup'

def files_under(folder: str):
    """Return a flat list of all regular files under `folder` (recursively)."""
    p = Path(folder)
    if not p.exists():
        return []
    return [str(f) for f in p.rglob('*') if f.is_file()]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # fine even if no python modules; harmless
    data_files=[
        # Register package with ament
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),

        # Include package.xml
        (f'share/{package_name}', ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Configs (both YAML and any RViz files you kept in config/)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),

        # RViz directory (if you’re also keeping separate RViz files here)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Recurse into description/ and worlds/ (include files only)
        (os.path.join('share', package_name, 'description'), files_under('description')),
        (os.path.join('share', package_name, 'worlds'),      files_under('worlds')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stenneth Swaby',
    maintainer_email='swabys1@tcnj.edu',
    description='Bringup package for running SLAM Toolbox with TurtleBot3 in Gazebo and RViz.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
