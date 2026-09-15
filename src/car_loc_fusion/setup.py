import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_loc_fusion'

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
    description='方法五: 結合所有感測器 (相機 + LiDAR + IMU + 輪速) 的融合定位',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fusion_localizer = car_loc_fusion.fusion_loc_node:main',
            'fusion_loc_eval = car_loc_fusion.evaluate:main',
        ],
    },
)
