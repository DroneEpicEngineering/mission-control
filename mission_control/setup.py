from setuptools import find_packages, setup

package_name = "mission_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/example_startup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dee",
    maintainer_email="k4aro0@gmail.com",
    description="Flow control of the mission",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            f"behaviour_tree = {package_name}.behaviour_tree:main"
        ],
    },
)
