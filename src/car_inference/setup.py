from setuptools import find_packages, setup

package_name = "car_inference"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            'share/' + package_name + '/resource',
            [
                'resource/best.onnx',
            ],
        ),
        (
            'share/' + package_name + '/config',
            [
                'config/camera_ground.yaml',
            ],
        ),
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "car_inference_node = car_inference.car_inference_node:main"
        ],
    },
)
