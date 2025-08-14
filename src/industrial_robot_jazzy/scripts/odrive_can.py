import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Bool, Float32


class ODriveCANNode(Node):
    """Simple node that communicates with an ODrive over CAN."""

    def __init__(self):
        super().__init__('odrive_can_node')
        try:
            self.bus = can.ThreadSafeBus('can0', bustype='socketcan')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to connect to CAN bus: {exc}')
            raise

        self.get_logger().info('ODrive CAN node started')
        self.timer = self.create_timer(1.0, self.send_commands)
        self.notifier = can.Notifier(self.bus, [self.receive_message])
        self.create_subscription(Bool,
                                 'odrive/calibrate_offset',
                                 self.calibrate_offset_callback,
                                 10)
        self.create_subscription(Float32,
                                 'odrive/vel_limit',
                                 self.vel_limit_callback,
                                 10)
        self.create_subscription(Float32,
                                 'odrive/current_limit',
                                 self.current_limit_callback,
                                 10)

    def send_commands(self):
        """Send periodic control and status request frames."""
        # Enter closed-loop control
        try:
            msg = can.Message(arbitration_id=0x207,
                              data=[0x07, 0x00, 0x00, 0x00],
                              is_extended_id=False)
            self.bus.send(msg)
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send control frame: {exc}')

        # Request encoder estimates (remote frame)
        try:
            req = can.Message(arbitration_id=0x009,
                              is_extended_id=False,
                              is_remote_frame=True,
                              dlc=8)
            self.bus.send(req)
        except can.CanError as exc:
            self.get_logger().error(f'Failed to request status: {exc}')

    def receive_message(self, msg: can.Message):
        """Handle incoming CAN frames."""
        if msg.arbitration_id == 0x009 and not msg.is_remote_frame:
            pos = int.from_bytes(msg.data[0:4], byteorder='little', signed=True) / 100000.0
            vel = int.from_bytes(msg.data[4:8], byteorder='little', signed=True) / 100000.0
            self.get_logger().info(f'Pos: {pos:.2f}, Vel: {vel:.2f}')
        else:
            self.get_logger().debug(f'Received CAN frame: {msg}')

    def calibrate_offset_callback(self, msg: Bool):
        """Request offset calibration when True is received."""
        if not msg.data:
            return
        try:
            frame = can.Message(arbitration_id=0x207,
                                data=[0x03, 0x00, 0x00, 0x00],
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info('Calibration command sent')
        except can.CanError as exc:
            self.get_logger().error(f'Calibration command failed: {exc}')

    def vel_limit_callback(self, msg: Float32):
        """Set the ODrive velocity limit."""
        vel = int(msg.data * 1000)
        data = vel.to_bytes(4, 'little', signed=True) + (0).to_bytes(4, 'little', signed=True)
        try:
            frame = can.Message(arbitration_id=0x00B,
                                data=data,
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set velocity limit to {msg.data}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set velocity limit: {exc}')

    def current_limit_callback(self, msg: Float32):
        """Set the ODrive current limit."""
        current = int(msg.data * 1000)
        data = current.to_bytes(4, 'little', signed=True) + (0).to_bytes(4, 'little', signed=True)
        try:
            frame = can.Message(arbitration_id=0x00C,
                                data=data,
                                is_extended_id=False)
            self.bus.send(frame)
            self.get_logger().info(f'Set current limit to {msg.data}')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to set current limit: {exc}')


def main():
    rclpy.init()
    node = ODriveCANNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()