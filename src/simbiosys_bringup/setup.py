import os
from glob import glob

from setuptools import find_packages, setup

package_name = "simbiosys_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SimBioSys Group 06",
    maintainer_email="group06@example.com",
    description="Laptop-side launch files for SimBioSys team packages.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wait_for_checkpoint_ready = simbiosys_bringup.wait_for_checkpoint_ready:main",
        ],
    },
)
