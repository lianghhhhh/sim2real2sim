import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sensor_mpc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.csv')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='MPC path tracking on sensor-based localization (fusion odom + wheel feedback), '
                'for the real car and for sim2real comparison',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sensor_mpc_node = sensor_mpc_pkg.main:main',
            'path_tracker = sensor_mpc_pkg.path_tracker_node:main',
        ],
    },
)
