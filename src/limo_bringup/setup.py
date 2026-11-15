from glob import glob
import os

from setuptools import find_packages, setup

package_name = "limo_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BE Groupe 4",
    maintainer_email="e35612u@univ-lorraine.fr",
    description="Launch files for the J1 lane follower.",
    license="Apache-2.0",
)
