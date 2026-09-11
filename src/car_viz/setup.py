import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_viz'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='Foxglove bridge launch + layout + /rgb JPEG compressor for the car.usd localization stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'image_compressor = car_viz.image_compressor:main',
        ],
    },
)
