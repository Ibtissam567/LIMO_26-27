from setuptools import find_packages, setup

package_name = "limo_planning"

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
    description="Planning package skeleton.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lane_planner = limo_planning.lane_planner:main",
            "trajectory_tester = limo_planning.trajectory_tester:main",
            "road_planner = limo_planning.road_planner:main",
        ],
    },
)
