from setuptools import find_packages, setup

# 仅 Python 包, 不引入 ROS 2 / Gazebo / PX4 / C++ 依赖.
# 详见 工程文档/低智能AI_Windows11无仿真无C++生成指南.md.

setup(
    name="reacquisition_first_ring_raw",
    version="0.1.0",
    description=(
        "第一环最简原始数据中间件: 纯 Python, 无 ROS 2 / Gazebo / PX4 / C++ 依赖. "
        "仅做原始数据封装、mock、JSONL 写入和格式校验."
    ),
    author="reacquisition-ws",
    license="Proprietary",
    package_dir={"": "."},
    packages=find_packages(exclude=["tests", "tests.*"]),
    python_requires=">=3.8",
    install_requires=[],
    extras_require={
        "dev": ["pytest>=7.0"],
    },
    include_package_data=True,
    zip_safe=False,
)
