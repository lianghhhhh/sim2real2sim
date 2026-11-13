from glob import glob

from setuptools import find_packages, setup

package_name = 'collect_car_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liangh',
    maintainer_email='selenahuang0218@gmail.com',
    description='動態模型訓練資料的激發駕駛 (Isaac 模擬, 輪速控制)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 只有駕駛, 不記錄; 收資料用 collect_car.launch.py (會一起開 collect_data_node)
            'control_car_node = collect_car_pkg.control_car_node:main',
        ],
    },
)
