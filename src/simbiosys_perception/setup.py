from setuptools import find_packages, setup

package_name = "simbiosys_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["models/*.pt"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SimBioSys Group 06",
    maintainer_email="group06@example.com",
    description="Sensor processing and plant/environment analysis for SimBioSys.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_detection_node = simbiosys_perception.obstacle_detection_node:main",
            "plant_analysis_node = simbiosys_perception.plant_analysis_node:main",
            "bug_detection_node = simbiosys_perception.bug_detection_node:main",
            "flower_detection_node = simbiosys_perception.flower_detection_node:main",
            "apriltag_detection_node = "
            "simbiosys_perception.apriltag_detection_node:main",
            "bin_wall_alignment_node = simbiosys_perception.bin_wall_alignment_node:main",
            "bed_side_alignment_node = simbiosys_perception.bed_side_alignment_node:main",
            "alignment_strafe_test_node = "
            "simbiosys_perception.alignment_strafe_test_node:main",
        ],
    },
)
