from launch import LaunchDescription
from launch.actions import LogInfo

from simbiosys_bringup.launch_utils import optional_xml_launch


def generate_launch_description():
    """Launch the MIRTE Master Gazebo simulation when it is installed."""
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "MIRTE Gazebo is reused from mirte_gazebo. Install it through "
                    "repos.repos, Pixi, and rosdep; do not copy MIRTE documentation "
                    "or MIRTE packages into this repository."
                )
            ),
            optional_xml_launch(
                "mirte_gazebo",
                ["launch", "gazebo_mirte_master_empty.launch.xml"],
                (
                    "mirte_gazebo is not installed. Install MIRTE simulation "
                    "dependencies through repos.repos/Pixi/rosdep, then rebuild."
                ),
            ),
        ]
    )
