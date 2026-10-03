from setuptools import find_packages, setup
import os
from glob import glob

package_name = "px4_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy>=1.22,<1.25"],
    zip_safe=True,
    maintainer="px4-bridge",
    maintainer_email="dev@example.com",
    description="Lightweight ROS2 PX4 control bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge = px4_bridge.main:main",
            "odom_bridge = px4_bridge.main:odom_bridge_main",
        ],
    },
)
