from glob import glob
from setuptools import setup


package_name = "reacquisition_first_ring_ros"

setup(
    name=package_name,
    version="0.1.0",
    py_modules=[
        "raw_frame_publisher_node",
        "target_observation_frame_node",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reacquisition-ws",
    maintainer_email="dev@example.invalid",
    description="ROS 2 adapter nodes for first-ring raw frame publishing.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "first_ring_raw_frame_node = raw_frame_publisher_node:main",
            "target_observation_frame_node = target_observation_frame_node:main",
        ],
    },
)
