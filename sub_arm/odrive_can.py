#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import struct
from std_msgs.msg import Bool, Float32


class ODriveCANNode(Node):
    def __init__(self):
        super().__init__('odrive_can_node')
        try:
            self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to connect to CAN bus: {exc}. CAN functionality disabled.')
            self.bus = None
            return

        self.get_logger().info('ODrive CAN node started')
        
        # Initialize state variables
        self.vel_limit = 400.0
        self.current_limit = 10.0
        self.rotate_timer = None
        
        # Setup ODrive configuration
        if self.bus is not None:
            self.set_axis_node_id(0)
            self.set_control_mode()
            self.set_limits(self.vel_limit, self.current_limit)
            self.send_velocity_setpoint(0.0)
            self.request_errors()
            self.notifier = can.Notifier(self.bus, [self.receive_message])
        else:
            self.get_logger().warn('CAN bus not available, skipping notifier setup')
        
        # Create subscriptions
        self.create_subscription(Bool, 'odrive/reset_errors', self.reset_errors_callback, 10)
        self.create_subscription(Float32, 'odrive/vel_limit', self.vel_limit_callback, 10)
        self.create_subscription(Float32, 'odrive/current_limit', self.current_limit_callback, 10)
        self.create_subscription(Float32, 'odrive/rotate_duration', self.rotate_callback, 10)
        self.create_subscription(Float32, 'odrive/velocity_setpoint', self.velocity_setpoint_callback, 10)

    def _send_can_command(self, arbitration_id: int, data: bytes, description: str):
        """Helper method to send CAN frames with error handling."""
        if self.bus is None:
            self.get_logger().warn(f'CAN bus not available, skipping {description}')
            return
        try:
            frame = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'{description} sent')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to {description}: {exc}')

    def set_axis_node_id(self, node_id: int):
        """Set ODrive axis node ID."""
        data = node_id.to_bytes(4, 'little')
        self._send_can_command(0x006, data, f'Set axis node ID to {node_id}')

    def set_control_mode(self, control_mode: int = 2, input_mode: int = 1):
        """Set ODrive control mode to velocity control with vel_ramp input.
        
        Args:
            control_mode: 2 = VELOCITY_CONTROL
            input_mode: 1 = VEL_RAMP
        """
        data = control_mode.to_bytes(4, 'little') + input_mode.to_bytes(4, 'little')
        self._send_can_command(0x0B, data, 'Set control mode to VELOCITY_CONTROL with VEL_RAMP')

    def set_limits(self, vel_limit: float, current_limit: float):
        """Set ODrive velocity and current limits."""
        vel_data = struct.pack('<f', vel_limit)
        current_data = struct.pack('<f', current_limit)
        data = vel_data + current_data
        self._send_can_command(0x0C, data, f'Set limits: vel={vel_limit}, current={current_limit}')

    def set_motor_config(self, pole_pairs: int = 7):
        """Set ODrive motor configuration.
        
        Args:
            pole_pairs: Number of pole pairs for BLDC motor
        """
        calibration_current = 10.0
        resistance_calib_max_voltage = 4.0
        phase_inductance = 0.0
        phase_resistance = 0.0
        
        data = (pole_pairs.to_bytes(4, 'little') +
                struct.pack('<f', calibration_current) +
                struct.pack('<f', resistance_calib_max_voltage) +
                struct.pack('<f', phase_inductance) +
                struct.pack('<f', phase_resistance))
        self._send_can_command(0x0E, data, f'Set motor config: pole_pairs={pole_pairs}')

    def set_encoder_mode(self, mode: int = 1):
        """Set ODrive encoder mode.
        
        Args:
            mode: 1 = LOCKIN_SPIN (for sensorless)
        """
        data = mode.to_bytes(4, 'little')
        self._send_can_command(0x0D, data, f'Set encoder mode to {mode}')

    def receive_message(self, msg: can.Message):
        """Handle incoming CAN frames."""
        if msg.arbitration_id == 0x009 and not msg.is_remote_frame:
            pos = int.from_bytes(msg.data[0:4], byteorder='little', signed=True) / 100000.0
            vel = int.from_bytes(msg.data[4:8], byteorder='little', signed=True) / 100000.0
            self.get_logger().info(f'Pos: {pos:.2f}, Vel: {vel:.2f}')
        elif msg.arbitration_id == 0x018 and not msg.is_remote_frame:
            error_code = int.from_bytes(msg.data[0:4], byteorder='little')
            self.get_logger().info(f'Error code: {error_code}')
        else:
            self.get_logger().debug(f'Received CAN frame: {msg}')

    def vel_limit_callback(self, msg: Float32):
        """Update velocity limit."""
        self.vel_limit = msg.data
        self.set_limits(self.vel_limit, self.current_limit)

    def current_limit_callback(self, msg: Float32):
        """Update current limit."""
        self.current_limit = msg.data
        self.set_limits(self.vel_limit, self.current_limit)

    def rotate_callback(self, msg: Float32):
        """Rotate the motor for the specified duration at 400 rad/s."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, ignoring rotate command')
            return
        duration = msg.data
        if duration <= 0:
            return
        self.send_velocity_setpoint(400.0)  # Rotate at 400 rad/s
        self.rotate_timer = self.create_timer(duration, self.stop_rotation)
        self.get_logger().info(f'Started rotation for {duration} seconds')

    def stop_rotation(self):
        """Stop the rotation by setting velocity to 0."""
        self.send_velocity_setpoint(0.0)
        if self.rotate_timer is not None:
            self.destroy_timer(self.rotate_timer)
            self.rotate_timer = None
        self.get_logger().info('Stopped rotation')

    def send_velocity_setpoint(self, vel: float):
        """Send open-loop velocity setpoint to ODrive."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, ignoring velocity setpoint')
            return
        
        data = struct.pack('<f', vel)
        self._send_can_command(0x200, bytes([0x0c]) + data, f'Open-loop velocity setpoint: {vel} rad/s')

    def request_errors(self):
        """Request error status from ODrive."""
        try:
            req = can.Message(arbitration_id=0x018,  # 0x018 for node 0 error request
                              is_remote_frame=True,
                              dlc=8)
            self.bus.send(req)
            self.get_logger().info('Requested error status')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to request errors: {exc}')

    def velocity_setpoint_callback(self, msg: Float32):
        """Set the velocity setpoint directly."""
        self.send_velocity_setpoint(msg.data)

    def reset_errors_callback(self, msg: Bool):
        """Reset errors on axis0."""
        if not msg.data:
            return
        data = bytes([0x03, 0x00, 0x00, 0x00])
        self._send_can_command(0x200, data, 'Reset errors')


def main():
    rclpy.init()
    node = ODriveCANNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()