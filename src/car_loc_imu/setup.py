import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_loc_imu'

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
    description='方法三: 只用 IMU 的慣性定位 (含抗漂移)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'imu_localizer = car_loc_imu.imu_loc_node:main',
            'imu_loc_eval = car_loc_imu.evaluate:main',
            'imu_fit_noise = car_loc_imu.fit_noise:main',
        ],
    },
)
