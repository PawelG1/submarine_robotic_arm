import rclpy
from rclpy.node import Node
import can
import struct
from std_msgs.msg import Bool, Float32


class ODriveCANNode(Node):

    def __init__(self): # Initialize the ODrive CAN node
        # Initialize CAN bus and ROS2 node
        super().__init__('odrive_can_node') 
        try:
            self.bus = can.ThreadSafeBus(channel='can0', interface='socketcan') # Connect to CAN bus
        except can.CanError as exc:
            self.get_logger().error(f'Failed to connect to CAN bus: {exc}')
            raise # Exit if CAN bus is not available

        self.get_logger().info('ODrive CAN node started') # Configure ODrive logger

        # Calibrate ODrive settings
        self.reset_errors()  # Clear any existing errors
        self.set_axis_node_id(0)  # Ensure node ID is 0
        self.set_control_mode()  # Set to velocity control with vel_ramp input
        self.set_limits(400.0, 10.0)  # Set velocity limit to 400 rad/s, current limit to 10A
        self.send_velocity_setpoint(0.0)  # Initialize to 0 velocity
        self.request_errors()  # Request error status on startup

        # Set up CAN message notifier and periodic command sender
        self.notifier = can.Notifier(self.bus, [self.receive_message]) # Set up CAN message notifier
        self.timer = self.create_timer(0.02, self.send_commands) # 50 Hz command sending

        self.get_logger().info('ODrive CAN node started successfully.')
        # Set up ROS2 subscriptions
        self.create_subscription(Bool,
                                 'odrive/calibrate_offset',
                                 self.calibrate_offset_callback,
                                 10) # Subscribe to calibrate offset command, Service callback
        self.create_subscription(Bool,
                                 'odrive/reset_errors',
                                 self.reset_errors_callback,
                                 10) # Subscribe to reset errors command, Service callback                 
        self.create_subscription(Float32,
                                 'odrive/velocity_setpoint',
                                 self.velocity_setpoint_callback,
                                 10) # Subscribe to velocity setpoint command

    # Define methods to configure ODrive via CAN

    # Reset ODrive errors
    def reset_errors(self):
        try:
            node_id = 0  # jeśli Twoja oś ma ID = 0
            arbitration_id = 0x018 + node_id # 0x018 for node 0, reset errors

            frame = can.Message(arbitration_id=arbitration_id,  # Standard protocol for Firmware ODrive 0.5.1 and later
                                is_extended_id=False,
                                dlc=0)

            self.bus.send(frame) # Send CAN frame to reset errors

            self.get_logger().info('Reset ODrive errors') # Log success

        except can.CanError as exc:
            self.get_logger().error(f'Failed to reset errors: {exc}') # Log failure

    # Set axis node ID
    def set_axis_node_id(self, node_id: int):
        data = node_id.to_bytes(4, 'little') # Prepare data to set node ID, Little-endian
        try:
            arbitration_id = 0x006 # Command Set Axis Node ID

            frame = can.Message(arbitration_id=arbitration_id,  # Command Set Axis Node ID
                                data=list(data),
                                is_extended_id=False)

            self.bus.send(frame) # Send CAN frame to set node ID

            self.get_logger().info(f'Set axis node ID to {node_id}') # Log success

        except can.CanError as exc:
            self.get_logger().error(f'Failed to set node ID: {exc}') # Log failure

    # Set control mode to VELOCITY_CONTROL with VEL_RAMP input
    def set_control_mode(self):
        control_mode = 2  # VELOCITY_CONTROL
        input_mode = 1    # VEL_RAMP

        data = control_mode.to_bytes(4, 'little') + input_mode.to_bytes(4, 'little') # Prepare data to set control mode, Little-endian

        try:
            node_id = 0  # jeśli Twoja oś ma ID = 0
            cmd_id = 0x0B  # CMD_SET_CONTROL_MODE
            arbitration_id = 0x200 * node_id + cmd_id # Calculate arbitration ID for your node and control mode

            # Prepare data to set control mode
            frame = can.Message(
                arbitration_id=arbitration_id, # Standard protocol for Firmware ODrive 0.5.1 and later
                data=list(data),
                is_extended_id=False
            )

            self.bus.send(frame) # Send CAN frame to set control mode

            self.get_logger().info(f'Set control mode to VELOCITY_CONTROL: {control_mode} + VEL_RAMP: {input_mode} (ID=0x{arbitration_id:X})')
            # Log success

        except can.CanError as exc:
            self.get_logger().error(f'Failed to set control mode: {exc}') # Log failure

    # Set velocity and current limits
    def set_limits(self, vel_limit: float, current_limit: float):

        # Set ODrive velocity and current limits
        vel_data = struct.pack('<f', vel_limit) # Prepare velocity limit data, float 32bit little-endian
        current_data = struct.pack('<f', current_limit) # Prepare current limit data, float 32bit little-endian
        data = vel_data + current_data # Combine velocity and current limit data, 8 bytes total

        try:
            node_id = 0  # jeśli Twoja oś ma ID = 0
            arbitration_id = 0x0C + node_id # Calculate arbitration ID for your node

            frame = can.Message(
                arbitration_id=arbitration_id,  # Standard protocol for Firmware ODrive 0.5.1 and later
                data=list(data),
                is_extended_id=False
            )

            self.bus.send(frame) # Send CAN frame to set limits

            self.get_logger().info(f'Set limits: velocity {vel_limit}, current {current_limit}')
            # Log success

        except can.CanError as exc:
            self.get_logger().error(f'Failed to set limits: {exc}') # Log failure

    # Request error status from ODrive
    def request_errors(self):
        try:
            node_id = 0  # jeśli Twoja oś ma ID = 0
            arbitration_id = 0x018 + node_id # 0x018 for node 0, error request

            req = can.Message(arbitration_id=arbitration_id,  # Standard protocol for Firmware ODrive 0.5.1 and later
                              is_remote_frame=True,
                              dlc=8)

            self.bus.send(req) # Send CAN frame to request errors

            self.get_logger().info('Requested error status') # Log success

        except can.CanError as exc:
            self.get_logger().error(f'Failed to request errors: {exc}') # Log failure

    # Send velocity setpoint to ODrive
    def send_velocity_setpoint(self, vel: float):
        data = struct.pack('<f', vel) # Prepare velocity setpoint data, float 32bit little-endian
        try:
            node_id = 0  # jeśli Twoja oś ma ID = 0
            arbitration_id = 0x200 * node_id  # Calculate arbitration ID for your node

            msg = can.Message(arbitration_id=arbitration_id,  # Standard protocol for Firmware ODrive 0.5.1 and later
                              data=[0x0c] + list(data),  # 0x0c for open-loop velocity
                              is_extended_id=False)
            self.bus.send(msg) # Send CAN frame with velocity setpoint

            self.get_logger().info(f'Sent open-loop velocity setpoint: {vel}') # Log success

        except can.CanError as exc:
            self.get_logger().error(f'Failed to send velocity setpoint: {exc}') # Log failure

# TODO: Implement return message handling
# TODO: Subscriptions, How they work etc
# TODO: Callbacks for subscriptions
# TODO: Evoking for different nodes
# TODO: Hexadecimal representation of CAN IDs


def main():
    rclpy.init() # Initialize ROS2
    node = ODriveCANNode() # Create ODrive CAN node instance
    try:
        rclpy.spin(node) # Keep node running
    except KeyboardInterrupt: 
        pass # Allow graceful shutdown on Ctrl+C
    finally:
        node.destroy_node() # Clean up node
        rclpy.shutdown() # Shutdown ROS2


if __name__ == '__main__':
    main() # Run main function if script is executed directly