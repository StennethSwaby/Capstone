from setuptools import setup
from glob import glob
import os

package_name = 'cartographer_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stenneth Swaby',
    maintainer_email='swabys1@tcnj.edu',
    description='Cartographer bringup (2D) for sim/robot',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)

