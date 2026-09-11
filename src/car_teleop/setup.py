import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='Teleoperation for the Isaac Sim car.usd vehicle',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = car_teleop.cmd_vel_bridge:main',
            'teleop_key = car_teleop.teleop_key:main',
        ],
    },
)
