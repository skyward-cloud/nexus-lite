from glob import glob
import os

from setuptools import setup

package_name = 'px4_sitl_gz_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join('launch', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PX4 Development Team',
    maintainer_email='development@px4.io',
    description='ROS 2 launch files for PX4 SITL + gz sim',
    license='BSD-3-Clause',
)
