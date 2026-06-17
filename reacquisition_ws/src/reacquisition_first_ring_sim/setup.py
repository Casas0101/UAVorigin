from glob import glob
from setuptools import setup


package_name = "reacquisition_first_ring_sim"

setup(
    name=package_name,
    version="0.1.0",
    py_modules=[
        "gazebo_state_publisher_node",
        "raw_frame_jsonl_recorder_node",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
        (f"share/{package_name}/missions", glob("missions/*.plan")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reacquisition-ws",
    maintainer_email="dev@example.invalid",
    description="Gazebo launch and data collection tools for first-ring raw frames.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "first_ring_sim_state_node = gazebo_state_publisher_node:main",
            "raw_frame_jsonl_recorder = raw_frame_jsonl_recorder_node:main",
        ],
    },
)
