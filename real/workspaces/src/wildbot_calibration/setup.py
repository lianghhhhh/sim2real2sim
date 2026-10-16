from glob import glob
from setuptools import find_packages, setup

package_name = "wildbot_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "ruamel.yaml", "numpy"],
    zip_safe=True,
    maintainer="SeanPai",
    maintainer_email="seanpai96@gmail.com",
    description="Interactive calibration scripts for the wildbot robot.",
    license="MIT",
    tests_require=[],
    entry_points={
        "console_scripts": [
            "imu_axis_sanity = wildbot_calibration.imu_axis_sanity:main",
            "imu_static_calibration = wildbot_calibration.imu_static_calibration:main",
            "wheel_linear_calibration = wildbot_calibration.wheel_linear_calibration:main",
            "wheel_angular_calibration = wildbot_calibration.wheel_angular_calibration:main",
            "imu_accel_two_face = wildbot_calibration.imu_accel_two_face:main",
            "run_all_calibrations = wildbot_calibration.run_all:main",
        ],
    },
)
