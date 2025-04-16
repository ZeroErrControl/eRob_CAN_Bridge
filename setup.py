from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'erob_can_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'can_bridge_node = erob_can_bridge.can_bridge_node:main',
        'position_control = erob_can_bridge.position_control:main',
        'speed_control = erob_can_bridge.speed_control:main',
        'direct_speed_control = erob_can_bridge.direct_speed_control:main',
        'direct_position_control = erob_can_bridge.direct_position_control:main',
        'test_position_control = erob_can_bridge.test_position_control:main',
        'multi_position_control = erob_can_bridge.multi_position_control:main',
        ],
    },
)
