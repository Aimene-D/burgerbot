from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_root = Path(__file__).resolve().parents[1]
    urdf_path = project_root / "urdf" / "burger_robot.urdf"
    ekf_params_path = project_root / "config" / "ekf.yaml"
    robot_description = urdf_path.read_text(encoding="utf-8")
    use_ekf = LaunchConfiguration("use_ekf")
    use_imu_filter = LaunchConfiguration("use_imu_filter")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="Start robot_localization EKF node",
    )

    use_imu_filter_arg = DeclareLaunchArgument(
        "use_imu_filter",
        default_value="true",
        description="Start imu_filter_madgwick to build /imu/data from /imu/raw and /imu/mag",
    )

    use_joint_state_publisher_arg = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="false",
        description="Start joint_state_publisher (optional for static visualization)",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_joint_state_publisher),
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[
            {
                "use_mag": False,
                "publish_tf": False,
                "gain": 0.01,
                "zeta": 0.0,
            }
        ],
        remappings=[
            ("/imu/data_raw", "/imu/raw"),
            ("/imu/mag", "/imu/mag"),
            ("/imu/data", "/imu/data"),
        ],
        condition=IfCondition(use_imu_filter),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[str(ekf_params_path)],
        condition=IfCondition(use_ekf),
    )

    return LaunchDescription([
        use_ekf_arg,
        use_imu_filter_arg,
        use_joint_state_publisher_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        imu_filter_node,
        ekf_node,
    ])