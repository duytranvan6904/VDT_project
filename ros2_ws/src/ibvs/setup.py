from setuptools import setup

package_name = 'ibvs'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ibvs.launch.py']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='vdt',
    maintainer_email='vdt@example.com',
    description='Image-based visual servoing for gimbal pitch and drone yaw',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ibvs_controller = ibvs.ibvs_controller:main',
        ],
    },
)