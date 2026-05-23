from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_joint_states_topic = LaunchConfiguration("moveit_joint_states_topic")

    moveit_config = (
        MoveItConfigsBuilder("mirte", package_name="mirte_moveit_config")
        .robot_description(file_path="config/mirte_master.urdf.xacro")
        .robot_description_semantic(file_path="config/mirte_master.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    joint_state_adapter = Node(
        package="simbiosys_arm",
        executable="moveit_joint_state_adapter_node",
        name="moveit_joint_state_adapter_node",
        output="screen",
        parameters=[
            {
                "input_topic": "/joint_states",
                "output_topic": moveit_joint_states_topic,
            }
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        remappings=[("/joint_states", moveit_joint_states_topic)],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "moveit_joint_states_topic",
                default_value="/simbiosys/moveit_joint_states",
            ),
            SetParameter(name="use_sim_time", value=use_sim_time),
            joint_state_adapter,
            move_group,
        ]
    )
