from setuptools import find_packages, setup

package_name = "simbiosys_behavior"

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
    description="Mission manager and state-machine logic for SimBioSys.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bed_side_controller_node = simbiosys_behavior.bed_side_controller_node:main",
            "mission_manager_node = simbiosys_behavior.mission_manager_node:main",
        ],
    },
)
