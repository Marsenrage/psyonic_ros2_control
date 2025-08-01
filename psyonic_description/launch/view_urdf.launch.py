from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    description_package = FindPackageShare('psyonic_description')
    urdf_file = PathJoinSubstitution([description_package, 'urdf', 'psyonic_standalone.urdf.xacro'])
    rviz_config_dir = PathJoinSubstitution([description_package, 'rviz', 'rviz.rviz'])
    robot_description = ParameterValue(
            Command(['xacro ', urdf_file, 
                    ' use_mock_psyonic:=true',
                     ]), value_type=str)
    use_gui = DeclareLaunchArgument('use_gui', default_value='false', description='Whether to launch the joint state publisher with GUI')
    launch_rviz = DeclareLaunchArgument('launch_rviz', default_value='true', description='Whether to launch rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='psyonic_rsp',
        namespace='left_hand',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),
        namespace='left_hand',
        name='psyonic_jsp_gui',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
        name='psyonic_jsp',
        namespace='left_hand',

    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='psyonic_rviz2',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )
    return LaunchDescription([
        use_gui,
        launch_rviz,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz_node,
    ])
