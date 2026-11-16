from glob import glob
import os

from setuptools import find_packages, setup

package_name = "limo_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BE Groupe 4",
    maintainer_email="e35612u@univ-lorraine.fr",
    description="Lane, obstacle, sign and pose perception for the LIMO.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lane_detector = limo_perception.lane_detector:main",
            "obstacle_detector = limo_perception.obstacle_detector:main",
            "sign_detector = limo_perception.sign_detector:main",
            "localization_node = limo_perception.localization:main",
        ],
    },
)
