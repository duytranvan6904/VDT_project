import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'landing_guidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vdt',
    maintainer_email='sashisupo@gmail.com',
    description='Precision landing: covariance gate, SMC guidance, touchdown detector',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'covariance_gate_node = landing_guidance.covariance_gate_node:main',
            'smc_guidance_node = landing_guidance.smc_guidance_node:main',
            'touchdown_detector_node = landing_guidance.touchdown_detector_node:main',
        ],
    },
)