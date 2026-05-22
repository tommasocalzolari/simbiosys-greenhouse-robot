from setuptools import find_packages, setup

package_name = "simbiosys_arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SimBioSys Group 06",
    maintainer_email="group06@example.com",
    description="Named arm poses and gripper commands for SimBioSys.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "named_joint_pose_node = simbiosys_arm.named_joint_pose_node:main",
            "gripper_client_node = simbiosys_arm.gripper_client_node:main",
        ],
    },
)
