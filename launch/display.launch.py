import os
from struct import pack
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Ścieżka do katalogu pakietu
    pkg_share = get_package_share_directory('sub_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'sub_arm.urdf')
    
    return LaunchDescription([
        #argument dla urdf
        DeclareLaunchArgument(
            'urdf_file', 
            default_value=urdf_file,
            description='Full path to the URDF file to load'
        ),
        #robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        # #joint state publisher - do not use
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        # nasz poprzedni node z markerm
        Node(
            package='sub_arm',
            executable='my_visu',
            name='my_visu_node',
            output='screen'
        ),
        # nasz poprzedni node z markerm targetu
        Node(
            package='sub_arm',
            executable='target_publisher',
            name='target_publisher_node',
            output='screen'
        ),
        # publisher stanów joint'ów (do IK)
        Node(
            package='sub_arm',
            executable='arm_state_publisher',
            name='arm_state_publisher_node',
            output='screen'
        ),
        # UI do zadawania pozycji end-effectora
        Node(
            package='sub_arm',
            executable='ik_ui',
            name='ik_ui_node',
            output='screen'
        ),
        Node(
            package='sub_arm',
            executable='odrive_can',
            name='odrive_can',
            output='screen'
        )
        #rviz2 - USUNIĘTY z powodu błędów biblioteki snap
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')],
        # ),
        ]
    )
