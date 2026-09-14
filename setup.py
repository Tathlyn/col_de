from setuptools import setup
from glob import glob
import os

package_name = "col_de"

setup(
    name=package_name,
    version="0.1.0",
    packages=["src", "src.demo"],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="col_de contributors",
    maintainer_email="contributors@example.com",
    description="Generic IK + collision-checked + trapezoidal-velocity joint-space planning framework for ROS2/MoveIt2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "demo_waypoints = src.demo.demo_waypoints:main",
        ],
    },
)