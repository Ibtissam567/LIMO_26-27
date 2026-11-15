from setuptools import find_packages, setup

package_name = "limo_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BE Groupe 4",
    maintainer_email="e35612u@univ-lorraine.fr",
    description="Lane following controller for the LIMO.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lane_follower = limo_control.lane_follower:main",
            "polar_controller = limo_control.polar_controller:main",
            "behavior_manager = limo_control.behavior_manager:main",
        ],
    },
)
