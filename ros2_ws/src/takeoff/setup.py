from setuptools import setup

package_name = 'takeoff'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vdt',
    maintainer_email='todo@example.com',
    description='One-shot takeoff command for PX4 over uXRCE-DDS',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'takeoff = takeoff.takeoff_cmd:main',
        ],
    },
)
