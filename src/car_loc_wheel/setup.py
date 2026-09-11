import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_loc_wheel'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='IMU + 四輪輪速的航位推算 (7 維 EKF, 含打滑偵測)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'wheel_localizer = car_loc_wheel.wheel_loc_node:main',
            'wheel_loc_eval = car_loc_wheel.evaluate:main',
        ],
    },
)
