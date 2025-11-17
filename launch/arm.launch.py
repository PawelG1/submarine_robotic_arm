#!/usr/bin/env python3
"""
Launch file dla systemu sterowania ramieniem robotycznym.
Uruchamia:
- WebSocket server (wserver_ros.py) - interfejs sterowania
- PCA9685 node (pca9685_node.py) - sterownik serw
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    
    return LaunchDescription([
        LogInfo(msg=['Starting Robotic Arm Control System...']),
        
        # Node 1: PCA9685 servo controller
        Node(
            package='sub_arm',  
            executable='pca9685_node',
            name='pca9685_servo_controller',
            output='screen',
            parameters=[{
                'frequency': 50,
                'min_pulse_us': 500.0,
                'max_pulse_us': 2500.0,
            }],
            emulate_tty=True,
        ),
        
        # Node 2: WebSocket server with work cycle manager
        Node(
            package='sub_arm',
            executable='wserver_ros',
            name='websocket_server_node',
            output='screen',
            emulate_tty=True,
        ),
        
        LogInfo(msg=['Robotic Arm Control System started successfully']),
        LogInfo(msg=['WebSocket server available at: ws://localhost:8765']),
        LogInfo(msg=['Servo control topic: /servo_command']),
    ])


if __name__ == '__main__':
    generate_launch_description()
