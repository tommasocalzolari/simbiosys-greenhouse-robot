import os
from glob import glob

from setuptools import find_packages, setup

package_name = "simbiosys_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "INTEGRATION_README.md"],
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SimBioSys Group 06",
    maintainer_email="group06@example.com",
    description="Lightweight mapping wrappers and slam_toolbox configuration for SimBioSys.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "checkpoint_navigator_node = simbiosys_mapping.checkpoint_navigator_node:main",
            "mapping_status_node = simbiosys_mapping.mapping_status_node:main",
            "getmap_node = simbiosys_mapping.getmap_node:main",
            "initial_pose_node = simbiosys_mapping.initial_pose_node:main",
            "map_annotation_node = simbiosys_mapping.map_annotation_node:main",
            "map_post_processor_node = simbiosys_mapping.map_post_processor_node:main",
        ],
    },
)
