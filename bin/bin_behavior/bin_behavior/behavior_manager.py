import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

OPEN_POS   = -math.pi / 2   # -1.5708 rad  (joint lower limit)
CLOSED_POS =  0.0            # joint upper limit
WAIT_S     =  3.0
DOCK_X, DOCK_Y, DOCK_YAW = 0.0, 0.0, 0.0


class BehaviorManager(Node):
    IDLE, OPENING, WAITING, CLOSING, DOCKING = range(5)

    def __init__(self):
        super().__init__('behavior_manager')
        cb = ReentrantCallbackGroup()
        self._state = self.IDLE
        self._nav_goal_handle = None
        self._wait_timer  = None
        self._close_timer = None

        self._cover_pub = self.create_publisher(
            Float64MultiArray, '/bin_cover_controller/commands', 10)
        self.create_subscription(
            Bool, '/bin_trigger', self._trigger_cb, 10, callback_group=cb)
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose', callback_group=cb)
        self.get_logger().info('BehaviorManager ready')

    # ── helpers ──────────────────────────────────────────────────────────

    def _set_cover(self, position: float):
        msg = Float64MultiArray()
        msg.data = [position]
        self._cover_pub.publish(msg)

    def _cancel_nav(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    def _send_dock_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose server not available')
            self._state = self.IDLE
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = DOCK_X
        goal.pose.pose.position.y = DOCK_Y
        goal.pose.pose.orientation.w = math.cos(DOCK_YAW / 2)
        goal.pose.pose.orientation.z = math.sin(DOCK_YAW / 2)
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Dock goal rejected')
            self._state = self.IDLE
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._nav_goal_handle = None
        self.get_logger().info('Docking complete — returning to IDLE')
        self._state = self.IDLE

    # ── state machine ────────────────────────────────────────────────────

    def _trigger_cb(self, msg: Bool):
        if not msg.data or self._state != self.IDLE:
            return
        self.get_logger().info('Trigger received — starting sequence')
        self._state = self.OPENING

        # Cancel any active navigation goal (lidar will tilt with the cover)
        self._cancel_nav()

        # Open lid
        self._set_cover(OPEN_POS)
        self.get_logger().info(f'Lid opening ({OPEN_POS:.4f} rad)')

        self._state = self.WAITING
        self._wait_timer = self.create_timer(WAIT_S, self._on_wait_done)

    def _on_wait_done(self):
        if self._state != self.WAITING:
            return
        # Destroy timer so it doesn't keep firing
        self.destroy_timer(self._wait_timer)
        self._wait_timer = None

        self._state = self.CLOSING
        self.get_logger().info('Closing lid')
        self._set_cover(CLOSED_POS)

        # Brief settle time before resuming navigation
        self._close_timer = self.create_timer(1.5, self._on_close_done)

    def _on_close_done(self):
        if self._state != self.CLOSING:
            return
        self.destroy_timer(self._close_timer)
        self._close_timer = None

        self._state = self.DOCKING
        self.get_logger().info('Lid closed — sending dock goal')
        self._send_dock_goal()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()
