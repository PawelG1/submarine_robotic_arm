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
            return  # Skip the rest of init

        self.get_logger().info('ODrive CAN node started')
        if self.bus is not None:
            self.set_axis_node_id(0)  # Ensure node ID is 0
            self.set_control_mode()  # Set to velocity control with vel_ramp input
            self.set_limits(400.0, 10.0)  # Set velocity limit to 400 rad/s, current limit to 10A
            self.send_velocity_setpoint(0.0)  # Initialize to 0 velocity
            self.request_errors()  # Request error status on startup
            self.notifier = can.Notifier(self.bus, [self.receive_message])
        else:
            self.get_logger().warn('CAN bus not available, skipping notifier setup')
        # self.create_subscription(Bool,
        #                          'odrive/calibrate_offset',
        #                          self.calibrate_offset_callback,
        #                          10)
        self.create_subscription(Bool,
                                 'odrive/reset_errors',
                                 self.reset_errors_callback,
                                 10)
        self.create_subscription(Float32,
                                 'odrive/vel_limit',
                                 self.vel_limit_callback,
                                 10)
        self.create_subscription(Float32,
                                 'odrive/current_limit',
                                 self.current_limit_callback,
                                 10)
        self.create_subscription(Float32,
                                 'odrive/rotate_duration',
                                 self.rotate_callback,
                                 10)
        self.create_subscription(Float32,
                                 'odrive/velocity_setpoint',
                                 self.velocity_setpoint_callback,
                                 10)
        self.cealibrate_offset = False
        self.vel_limit = 1000.0  # Default velocity limit
        self.current_limit = 10.0  # Default current limit
        self.rotate_timer = None

    def set_control_mode(self):
        """Set ODrive control mode to velocity control with vel_ramp input."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping control mode setup')
            return
        control_mode = 2  # VELOCITY_CONTROL
        input_mode = 1    # VEL_RAMP
        data = control_mode.to_bytes(4, 'little') + input_mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x200,
                                data=[0x0B] + list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info('Set control mode to velocity control with vel_ramp input')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set control mode: {exc}')

    def set_axis_node_id(self, node_id: int):
        """Set ODrive axis node ID."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping node ID setup')
            return
        data = node_id.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x006,  # Command 0x006
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set axis node ID to {node_id}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set node ID: {exc}')

    def set_control_mode(self):
        """Set ODrive control mode to velocity control with vel_ramp input."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping control mode setup')
            return
        control_mode = 2  # VELOCITY_CONTROL
        input_mode = 1    # VEL_RAMP
        data = control_mode.to_bytes(4, 'little') + input_mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x0B,  # Command 0x0B for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info('Set control mode to velocity control with vel_ramp input')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set control mode: {exc}')

    def set_limits(self, vel_limit: float, current_limit: float):
        """Set ODrive velocity and current limits."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping limits setup')
            return
        vel_data = struct.pack('<f', vel_limit)
        current_data = struct.pack('<f', current_limit)
        data = vel_data + current_data
        try:
            frame = can.Message(arbitration_id=0x0C,  # Command 0x0C for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set limits: velocity {vel_limit}, current {current_limit}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set limits: {exc}')

    def set_motor_config(self, pole_pairs: int):
        """Set ODrive motor configuration."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping motor config')
            return
        calibration_current = 10.0
        resistance_calib_max_voltage = 4.0
        phase_inductance = 0.0
        phase_resistance = 0.0
        data = (pole_pairs.to_bytes(4, 'little') +
                struct.pack('<f', calibration_current) +
                struct.pack('<f', resistance_calib_max_voltage) +
                struct.pack('<f', phase_inductance) +
                struct.pack('<f', phase_resistance))
        try:
            frame = can.Message(arbitration_id=0x0E,  # Command 0x0E for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set motor config: pole_pairs={pole_pairs}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set motor config: {exc}')

    def set_encoder_mode(self, mode: int):
        """Set ODrive encoder mode."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping encoder mode')
            return
        data = mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x0D,  # Command 0x0D for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set encoder mode to {mode}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set encoder mode: {exc}')

    def set_control_mode(self):
        """Set ODrive control mode to velocity control with vel_ramp input."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping control mode setup')
            return
        control_mode = 2  # VELOCITY_CONTROL
        input_mode = 1    # VEL_RAMP
        data = control_mode.to_bytes(4, 'little') + input_mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x0B,  # Command 0x0B for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info('Set control mode to velocity control with vel_ramp input')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set control mode: {exc}')

    def set_limits(self, vel_limit: float, current_limit: float):
        """Set ODrive velocity and current limits."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping limits setup')
            return
        vel_data = struct.pack('<f', vel_limit)
        current_data = struct.pack('<f', current_limit)
        data = vel_data + current_data
        try:
            frame = can.Message(arbitration_id=0x0C,  # Command 0x0C for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set limits: velocity {vel_limit}, current {current_limit}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set limits: {exc}')

    def set_motor_config(self, pole_pairs: int):
        """Set ODrive motor configuration."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping motor config')
            return
        calibration_current = 10.0
        resistance_calib_max_voltage = 4.0
        phase_inductance = 0.0
        phase_resistance = 0.0
        data = (pole_pairs.to_bytes(4, 'little') +
                struct.pack('<f', calibration_current) +
                struct.pack('<f', resistance_calib_max_voltage) +
                struct.pack('<f', phase_inductance) +
                struct.pack('<f', phase_resistance))
        try:
            frame = can.Message(arbitration_id=0x0E,  # Command 0x0E for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set motor config: pole_pairs={pole_pairs}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set motor config: {exc}')

    def set_encoder_mode(self, mode: int):
        """Set ODrive encoder mode."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping encoder mode')
            return
        data = mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x0D,  # Command 0x0D for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set encoder mode to {mode}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set encoder mode: {exc}')

    def set_control_mode(self):
        """Set ODrive control mode to velocity control with vel_ramp input."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping control mode setup')
            return
        control_mode = 2  # VELOCITY_CONTROL
        input_mode = 1    # VEL_RAMP
        data = control_mode.to_bytes(4, 'little') + input_mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x0B,  # Command 0x0B for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info('Set control mode to velocity control with vel_ramp input')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set control mode: {exc}')

    def set_limits(self, vel_limit: float, current_limit: float):
        """Set ODrive velocity and current limits."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping limits setup')
            return
        vel_data = struct.pack('<f', vel_limit)
        current_data = struct.pack('<f', current_limit)
        data = vel_data + current_data
        try:
            frame = can.Message(arbitration_id=0x0C,  # Command 0x0C for node 0
                                data=list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set limits: velocity {vel_limit}, current {current_limit}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set limits: {exc}')

    def set_motor_config(self, pole_pairs: int):
        """Set ODrive motor configuration."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping motor config')
            return
        calibration_current = 10.0
        resistance_calib_max_voltage = 4.0
        phase_inductance = 0.0
        phase_resistance = 0.0
        data = (pole_pairs.to_bytes(4, 'little') +
                struct.pack('<f', calibration_current) +
                struct.pack('<f', resistance_calib_max_voltage) +
                struct.pack('<f', phase_inductance) +
                struct.pack('<f', phase_resistance))
        try:
            frame = can.Message(arbitration_id=0x200,
                                data=[0x0E] + list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set motor config: pole_pairs={pole_pairs}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set motor config: {exc}')

    def set_encoder_mode(self, mode: int):
        """Set ODrive encoder mode."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping encoder mode')
            return
        data = mode.to_bytes(4, 'little')
        try:
            frame = can.Message(arbitration_id=0x200,
                                data=[0x0D] + list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set encoder mode to {mode}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set encoder mode: {exc}')

    def set_limits(self, vel_limit: float, current_limit: float):
        """Set ODrive velocity and current limits."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, skipping limits setup')
            return
        vel_data = struct.pack('<f', vel_limit)
        current_data = struct.pack('<f', current_limit)
        data = vel_data + current_data
        try:
            frame = can.Message(arbitration_id=0x200,
                                data=[0x0C] + list(data),
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set limits: velocity {vel_limit}, current {current_limit}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set limits: {exc}')
#         """Send periodic control and status request frames."""
#         # Enter closed-loop control
#         try:
#             msg = can.Message(arbitration_id=0x207,
#                               data=[0x07, 0x00, 0x00, 0x00],
#                               is_extended_id=False)
#             self.bus.send(msg)
#         except can.CanError as exc:
#             self.get_logger().error(f'Failed to send control frame: {exc}')

#         # Request encoder estimates (remote frame)
#         try:
#             req = can.Message(arbitration_id=0x009,
#                               is_extended_id=False,
#                               is_remote_frame=True,
#                               dlc=8)
#             self.bus.send(req)
#         except can.CanError as exc:
#             self.get_logger().error(f'Failed to request status: {exc}')

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
        """Set the ODrive velocity limit."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, ignoring velocity limit command')
            return
        self.vel_limit = msg.data
        self.set_limits(self.vel_limit, self.current_limit)

    def current_limit_callback(self, msg: Float32):
        """Set the ODrive current limit."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, ignoring current limit command')
            return
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
        data = struct.pack('<f', vel)
        try:
            msg = can.Message(arbitration_id=0x200,  # 0x200 for node 0
                              data=[0x0c] + list(data),  # 0x0c for open-loop velocity
                              is_extended_id=False)
            self.bus.send(msg)
            self.get_logger().info(f'Sent open-loop velocity setpoint: {vel}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send velocity setpoint: {exc}')

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
        self.get_logger().info(f'Set velocity setpoint to {msg.data}')

    def reset_errors_callback(self, msg: Bool):
        """Reset errors on axis0."""
        if self.bus is None:
            self.get_logger().warn('CAN bus not available, ignoring reset errors command')
            return
        if not msg.data:
            return
        try:
            frame = can.Message(arbitration_id=0x200,
                                data=[0x03, 0x00, 0x00, 0x00],
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info('Sent reset errors command')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to reset errors: {exc}')


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