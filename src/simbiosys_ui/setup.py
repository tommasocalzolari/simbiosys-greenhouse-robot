from setuptools import find_packages, setup

package_name = "simbiosys_ui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        package_name: ["config/*.json"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["simbiosys_ui/config/rosTopics.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SimBioSys Group 06",
    maintainer_email="group06@example.com",
    description="Operator interface, teleop hooks, and dashboard logic for SimBioSys.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ui_node = simbiosys_ui.ui_node:main",
            "teleop_interface_node = simbiosys_ui.teleop_interface_node:main",
        ],
    },
)
