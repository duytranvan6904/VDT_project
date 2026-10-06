import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ekf_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="Target state EKF adapter for ArUco H-Pad measurements",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_node = ekf_adapter.ekf_node:main",
            "odom_tf_node = ekf_adapter.odom_tf_node:main",
        ],
    },
)
