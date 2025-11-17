#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import board
import busio
from adafruit_pca9685 import PCA9685

DEFAULT_FREQUENCY = 50
DEFAULT_MIN_PULSE_US = 500.0
DEFAULT_MAX_PULSE_US = 2500.0

class PCA9685Node(Node):
    """
    Node ROS2 który nasłuchuje na topicu /servo_command (Vector3) i ustawia
    odpowiedni sygnał PWM na wskazanym kanale PCA9685.

    Format wiadomości (geometry_msgs/Vector3):
    - x: channel (int, np. 0..15)
    - y: angle (float, stopnie 0..180)
    - z: zarezerwowane (nieużywane)

    Przykład publikacji:
    ros2 topic pub --once /servo_command geometry_msgs/msg/Vector3 "{x: 3.0, y: 90.0, z: 0.0}"
    """

    def __init__(self):
        super().__init__('pca9685_node')
        # Parametry (można zmienić z ROS2 param)
        self.declare_parameter('frequency', DEFAULT_FREQUENCY)  # typowo 50Hz dla serw
        self.declare_parameter('min_pulse_us', DEFAULT_MIN_PULSE_US)
        self.declare_parameter('max_pulse_us', DEFAULT_MAX_PULSE_US)

        freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.min_us = self.get_parameter('min_pulse_us').get_parameter_value().double_value
        self.max_us = self.get_parameter('max_pulse_us').get_parameter_value().double_value

        # Inicjalizacja I2C i PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = freq

        self.get_logger().info(f'PCA9685 zainicjalizowany, freq={freq} Hz')

        # Subskrypcja: nowy prosty format Vector3 (x=channel, y=angle)
        self.command_subscription = self.create_subscription(
            Vector3,
            'servo_command',
            self.set_servo_angle,
            10
        )
        self.command_subscription

    def set_servo_angle(self, msg: Vector3):
        """
        Nowy format sterowania: Vector3
        x -> channel (int), y -> angle (0..180)
        z -> (opcjonalnie) zarezerwowane
        """
        try:
            channel = int(msg.x)
        except Exception:
            self.get_logger().error(f'Niepoprawny channel w wiadomości: {msg.x}')
            return

        # validacja kanału (PCA9685 ma zwykle 0..15)
        max_channels = len(self.pca.channels)
        if channel < 0 or channel >= max_channels:
            self.get_logger().error(f'Nieprawidłowy channel: {channel}. Dozwolone 0..{max_channels-1}')
            return

        angle = float(msg.y)
        angle = max(0.0, min(180.0, angle))  # clamp 0–180
        duty = self.angle_to_duty_cycle(angle)
        self.set_channel_duty(channel, duty)
        self.get_logger().info(f'channel={channel} kąt={angle:.1f}°, duty={duty:.4f}')

    def angle_to_duty_cycle(self, angle: float) -> float:
        """
        Konwersja kąta [0–180] na wypełnienie [0.0–1.0]
        przy założeniu impulsu 1–2ms przy 50Hz.
        """
        # długość okresu w mikrosekundach przy danej częstotliwości
        period_us = 1_000_000.0 / self.pca.frequency

        pulse_us = self.min_us + (self.max_us - self.min_us) * (angle / 180.0)
        duty_cycle = pulse_us / period_us  # 0.0–1.0

        # zabezpieczenie
        duty_cycle = max(0.0, min(1.0, duty_cycle))
        return duty_cycle

    def set_channel_duty(self, channel: int, duty_cycle: float):
        """
        Ustawia wypełnienie kanału PCA9685.
        """
        # PCA9685 ma 12-bitowe rozdzielczości (0–4095)
        value = int(duty_cycle * 0xFFFF)  # biblioteka używa 16-bitów
        self.pca.channels[channel].duty_cycle = value

    def testAllServos(self, timeout):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pca.deinit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
