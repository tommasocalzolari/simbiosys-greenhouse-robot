from setuptools import find_packages, setup

package_name = "simbiosys_base"

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
    description="High-level base and navigation commands for SimBioSys.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_generation_node = simbiosys_base.path_generation_node:main",
            "path_execution_node = simbiosys_base.path_execution_node:main",
        ],
    },
)
