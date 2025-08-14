#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import Bool, Float32


class ODriveRvizInterface(Node):
    """Interactive marker interface to control ODrive from RViz."""

    def __init__(self):
        super().__init__('odrive_rviz_interface')
        self.server = InteractiveMarkerServer(self, 'odrive_controls')
        self.menu_handler = MenuHandler()
        self.calib_pub = self.create_publisher(Bool, 'odrive/calibrate_offset', 10)
        self.vel_pub = self.create_publisher(Float32, 'odrive/vel_limit', 10)

        self._init_menu()
        self._create_marker()
        self.server.applyChanges()

    def _init_menu(self):
        self.menu_handler.insert('Calibrate Offset', callback=self._calibrate_cb)
        vel_menu = self.menu_handler.insert('Velocity Limit')
        for value in [5.0, 10.0]:
            self.menu_handler.insert(f'{value} rad/s', parent=vel_menu,
                                     callback=self._make_vel_cb(value))

    def _create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.name = 'odrive_control'
        int_marker.description = 'ODrive Control'

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.name = 'button'

        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.g = 1.0
        marker.color.a = 1.0

        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker)
        self.menu_handler.apply(self.server, int_marker.name)

    def _calibrate_cb(self, feedback):
        self.get_logger().info('Offset calibration requested')
        self.calib_pub.publish(Bool(data=True))

    def _make_vel_cb(self, value):
        def cb(feedback):
            self.get_logger().info(f'Setting velocity limit to {value}')
            self.vel_pub.publish(Float32(data=value))
        return cb


def main():
    rclpy.init()
    node = ODriveRvizInterface()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
