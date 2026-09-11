import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_loc_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 模型檔沒有跟著 repo 走 (9.7 MB), 有放才裝; 見 README「模型放哪」
        (os.path.join('share', package_name, 'resource'), glob('resource/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='方法一: 只用天花板相機 + YOLO 的車輛定位',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera_localizer = car_loc_camera.camera_loc_node:main',
            'camera_loc_eval = car_loc_camera.evaluate:main',
        ],
    },
)
