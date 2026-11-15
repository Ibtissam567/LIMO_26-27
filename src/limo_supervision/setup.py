from setuptools import find_packages, setup

package_name = "limo_supervision"

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
    description="Supervision package skeleton.",
    license="Apache-2.0",
)
