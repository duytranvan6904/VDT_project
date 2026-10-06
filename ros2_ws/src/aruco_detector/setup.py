from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "aruco_detector"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "aruco_node = aruco_detector.aruco_node:main",
            "depth_to_image_node = aruco_detector.depth_to_image_node:main",
        ],
    },
)
