import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 地圖跟著 package 一起裝, 節點預設就去 share 裡找
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*.npz') + glob('maps/*.pgm') + glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='LiDAR + IMU localization for the Isaac Sim car.usd scene',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'car_localizer = car_localization.localizer:main',
            'localization_eval = car_localization.evaluate:main',
            'fake_isaac = car_localization.fake_isaac:main',
        ],
    },
)
