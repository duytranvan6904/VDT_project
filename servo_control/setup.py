from setuptools import find_packages, setup

package_name = 'servo_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mier',
    maintainer_email='mier@example.com',
    description='Servo PWM control cho gimbal actuator (MG90S, open-loop)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'servo_node = servo_control.servo_node:main',
        ],
    },
)