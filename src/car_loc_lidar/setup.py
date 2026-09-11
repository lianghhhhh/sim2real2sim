import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_loc_lidar'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 建好的地圖跟著 package 一起裝, 節點預設就去 share 裡找
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.npz') + glob('maps/*.pgm') + glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='方法二: 只用 LiDAR 的建圖與定位',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'lidar_localizer = car_loc_lidar.lidar_loc_node:main',
            'lidar_odometry = car_loc_lidar.scan_odom_node:main',
            'lidar_loc_eval = car_loc_lidar.evaluate:main',
        ],
    },
)
