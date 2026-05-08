import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    XMLLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_available(package_name: str) -> bool:
    try:
        get_package_prefix(package_name)
    except PackageNotFoundError:
        return False
    return True


def optional_node(package: str, executable: str, missing_message: str, **kwargs):
    if not package_available(package):
        return LogInfo(msg=missing_message)
    return Node(package=package, executable=executable, **kwargs)


def optional_python_launch(
    package: str,
    relative_path: list[str],
    missing_message: str,
    launch_arguments=None,
):
    if not package_available(package):
        return LogInfo(msg=missing_message)

    source = PythonLaunchDescriptionSource(
        [FindPackageShare(package), "/" + os.path.join(*relative_path)]
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(launch_arguments or {}).items(),
    )


def optional_xml_launch(
    package: str,
    relative_path: list[str],
    missing_message: str,
    launch_arguments=None,
):
    if not package_available(package):
        return LogInfo(msg=missing_message)

    source = XMLLaunchDescriptionSource(
        [FindPackageShare(package), "/" + os.path.join(*relative_path)]
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(launch_arguments or {}).items(),
    )
