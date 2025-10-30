import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ścieżka do katalogu pakietu
    pkg_share = get_package_share_directory('sub_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'sub_arm.urdf')
    
    return LaunchDescription([
        #robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='sub_arm',
            executable='odrive_can',
            name='odrive_can',
            output='screen'
        )
        ]
    )
