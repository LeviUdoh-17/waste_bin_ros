"""Republish the laser scan only while the bin lid is closed.

The LD19 is bolted to the lid (bin_cover_link -> lidar_link is fixed, but
bin_top_link -> bin_cover_link is the revolute bin_cover_joint), so opening the
lid pitches the scanner by up to 90 degrees. The scan plane then sweeps floor and
ceiling instead of the room, which corrupts the slam_toolbox map, marks phantom
obstacles in both Nav2 costmaps and raytrace-clears real ones.

This node sits between the driver and everything downstream: the driver publishes
/lidar_raw, and /lidar only carries scans taken while the lid was down.

The settle delay matters. There is no lid encoder, so bin_hardware echoes the
commanded position back as state immediately, while the firmware sweeps the servo
over roughly 300 ms. Gating on the reported joint angle alone would cut the scans
correctly on opening but resume them before the lid had physically come to rest.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')

        self.declare_parameter('input_topic', '/lidar_raw')
        self.declare_parameter('output_topic', '/lidar')
        self.declare_parameter('joint_name', 'bin_cover_joint')
        self.declare_parameter('closed_position', 0.0)
        self.declare_parameter('closed_tolerance', 0.05)   # rad
        self.declare_parameter('settle_time', 0.5)         # s
        self.declare_parameter('joint_timeout', 2.0)       # s

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        self.closed_position = self.get_parameter('closed_position').value
        self.closed_tolerance = self.get_parameter('closed_tolerance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.joint_timeout = self.get_parameter('joint_timeout').value

        # None until the first joint state arrives, so we fail closed at startup.
        self._closed_since = None
        self._last_joint_msg = None
        self._passing = None  # tri-state so the first decision always logs

        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10)

        # Also watch the command, not just the reported state. /joint_states only
        # updates at the controller_manager rate, so gating on it alone lets one scan
        # through after an open command — by which point the servo has already swept
        # a good fraction of its travel. Watching the command shuts the gate at once.
        self.create_subscription(
            Float64MultiArray, '/bin_cover_controller/commands',
            self._command_callback, 10)

        # Sensor QoS on both sides: the LD19 driver publishes best-effort, and the
        # downstream displays and slam_toolbox are already set up for that profile.
        self.sub = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'scan_gate: {input_topic} -> {output_topic}, gated on '
            f'{self.joint_name} within {self.closed_tolerance} rad of '
            f'{self.closed_position} for {self.settle_time}s')

    def _joint_callback(self, msg: JointState):
        try:
            i = msg.name.index(self.joint_name)
        except ValueError:
            return
        if i >= len(msg.position):
            return

        self._last_joint_msg = self.get_clock().now()
        is_closed = abs(msg.position[i] - self.closed_position) <= self.closed_tolerance

        if not is_closed:
            self._closed_since = None
        elif self._closed_since is None:
            self._closed_since = self._last_joint_msg

    def _command_callback(self, msg: Float64MultiArray):
        if not msg.data:
            return
        if abs(msg.data[0] - self.closed_position) > self.closed_tolerance:
            # Lid was told to move off closed — stop passing scans immediately and let
            # the joint-state path restart the settle timer once it is back down.
            self._closed_since = None

    def _gate_open(self) -> bool:
        """True when scans may pass: lid reported closed, settled, and not stale."""
        now = self.get_clock().now()

        # No joint state at all, or it has gone quiet — fail closed rather than
        # trust a stale reading.
        if self._last_joint_msg is None:
            return False
        if (now - self._last_joint_msg).nanoseconds * 1e-9 > self.joint_timeout:
            return False

        if self._closed_since is None:
            return False
        return (now - self._closed_since).nanoseconds * 1e-9 >= self.settle_time

    def _scan_callback(self, msg: LaserScan):
        passing = self._gate_open()

        if passing != self._passing:
            self._passing = passing
            if passing:
                self.get_logger().info('lid closed and settled — passing scans')
            else:
                self.get_logger().warn('lid not closed — blocking scans')

        if passing:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
