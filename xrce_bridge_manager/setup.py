from setuptools import find_packages, setup

package_name = 'xrce_bridge_manager'

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
    description='Khoi dong va giam sat MicroXRCEAgent, bao cao trang thai ket noi PX4',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'xrce_bridge_node = xrce_bridge_manager.xrce_node:main',
        ],
    },
)