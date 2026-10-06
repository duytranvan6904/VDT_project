import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'apf_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='VDT',
    maintainer_email='dev@vdt.local',
    description='APF / Improved APF obstacle-avoidance planner for H-Pad follow and approach.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'planner_node = apf_planner.planner_node:main',
            'pointcloud_generator = apf_planner.pointcloud_generator:main',
            'planner_merge_node = apf_planner.planner_merge_node:main',
        ],
    },
)
