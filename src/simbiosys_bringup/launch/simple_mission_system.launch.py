from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    simulation = LaunchConfiguration("simulation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    annotations_file = LaunchConfiguration("annotations_file")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    start_rviz = LaunchConfiguration("start_rviz")
    start_ui = LaunchConfiguration("start_ui")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    surface_region = LaunchConfiguration("surface_region")
    enable_alignment_motion = LaunchConfiguration("enable_alignment_motion")
    scan_position_dry_run = LaunchConfiguration("scan_position_dry_run")
    plant_analysis_dry_run = LaunchConfiguration("plant_analysis_dry_run")
    flower_model_path = LaunchConfiguration("flower_model_path")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("simbiosys_mapping"),
                    "launch",
                    "navigation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "simulation": simulation,
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "start_rviz": start_rviz,
            "cmd_vel_topic": cmd_vel_topic,
            "odom_topic": odom_topic,
            "scan_topic": scan_topic,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value="maps/mirte_map.yaml"),
            DeclareLaunchArgument(
                "annotations_file",
                default_value="maps/mirte_map_annotations.json",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/depth/image_raw",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/mirte_base_controller/odom",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/mirte_base_controller/cmd_vel",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description=(
                    "Start RViz with Nav2. Off by default for mission "
                    "stability."
                ),
            ),
            DeclareLaunchArgument("start_ui", default_value="true"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument(
                "surface_region",
                default_value="front",
                description=(
                    "Laser region used for final checkpoint alignment."
                ),
            ),
            DeclareLaunchArgument(
                "enable_alignment_motion",
                default_value="false",
                description=(
                    "Allow the alignment controller to publish physical "
                    "motion."
                ),
            ),
            DeclareLaunchArgument(
                "scan_position_dry_run",
                default_value="true",
                description=(
                    "Skip physical final alignment while validating the route."
                ),
            ),
            DeclareLaunchArgument(
                "plant_analysis_dry_run",
                default_value="true",
                description=(
                    "Return a synthetic analysis result without loading YOLO."
                ),
            ),
            DeclareLaunchArgument(
                "flower_model_path",
                default_value="models/flower_model.pt",
            ),
            navigation,
            Node(
                package="simbiosys_perception",
                executable="bed_side_alignment_node",
                name="bed_side_alignment_node",
                output="screen",
                parameters=[
                    {
                        "scan_topic": scan_topic,
                        "surface_region": surface_region,
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="simbiosys_behavior",
                executable="scan_pose_controller_node",
                name="scan_pose_controller_node",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "enable_motion": ParameterValue(
                            enable_alignment_motion,
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="simbiosys_perception",
                executable="flower_detection_node",
                name="flower_detection_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "depth_topic": depth_topic,
                        "model_path": flower_model_path,
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="simbiosys_behavior",
                executable="simple_mission_manager_node",
                name="simple_mission_manager_node",
                output="screen",
                parameters=[
                    {
                        "annotations_file": annotations_file,
                        "cmd_vel_topic": cmd_vel_topic,
                        "scan_position_dry_run": ParameterValue(
                            scan_position_dry_run,
                            value_type=bool,
                        ),
                        "plant_analysis_dry_run": ParameterValue(
                            plant_analysis_dry_run,
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="simbiosys_ui",
                executable="ui_node",
                name="ui_node",
                output="screen",
                condition=IfCondition(start_ui),
                parameters=[
                    {
                        "image_topic": image_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "web_host": web_host,
                        "web_port": web_port,
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
