"""Drive a route from waypoints.yaml using Nav2's waypoint_follower.

Runs on the laptop alongside laptop_nav_waypoints.launch.py. The route is handed to
the waypoint_follower action server, so the WaitAtWaypoint executor configured in
nav2_params.yaml applies at every stop.

The route is split into segments at each waypoint marked `open_lid: true`. Nav2 has
no hook for actuating the lid mid-route, and the lid must not open while the robot is
moving anyway: the LD19 is mounted on it, so scan_gate blocks /lidar for as long as
the lid is up and Nav2 is driving blind until it comes back.
"""

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

OPEN_POS = -math.pi / 2   # bin_cover_joint lower limit
CLOSED_POS = 0.0          # upper limit

# Must comfortably exceed the firmware's ~300 ms servo sweep plus scan_gate's
# settle_time, or navigation resumes before /lidar does.
CLOSE_SETTLE_S = 2.0


def _pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


class WaypointRunner(Node):
    def __init__(self):
        super().__init__('waypoint_runner')

        default_file = os.path.join(
            get_package_share_directory('bin_bringup'), 'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)

        self.cover_pub = self.create_publisher(
            Float64MultiArray, '/bin_cover_controller/commands', 10)

    def load_route(self):
        path = self.get_parameter('waypoints_file').value
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.get_logger().info(f'Loaded {path}')
        return (cfg.get('waypoints', []),
                bool(cfg.get('loop', False)),
                float(cfg.get('lid_dwell', 3.0)))

    def cycle_lid(self, dwell):
        msg = Float64MultiArray()
        self.get_logger().info(f'Opening lid, holding {dwell}s')
        msg.data = [OPEN_POS]
        self.cover_pub.publish(msg)
        time.sleep(dwell)

        self.get_logger().info('Closing lid')
        msg.data = [CLOSED_POS]
        self.cover_pub.publish(msg)
        # Wait for the servo to travel and scan_gate to re-open /lidar before moving.
        time.sleep(CLOSE_SETTLE_S)


def segment(waypoints):
    """Split the route so each segment ends on a lid stop (or at the route end)."""
    seg = []
    for wp in waypoints:
        seg.append(wp)
        if wp.get('open_lid', False):
            yield seg, True
            seg = []
    if seg:
        yield seg, False


def main(args=None):
    rclpy.init(args=args)
    runner = WaypointRunner()
    nav = BasicNavigator()

    try:
        waypoints, loop, lid_dwell = runner.load_route()
        if not waypoints:
            runner.get_logger().error('No waypoints in file — nothing to do')
            return

        runner.get_logger().info('Waiting for Nav2 to become active...')
        nav.waitUntilNav2Active()

        while rclpy.ok():
            for seg, ends_with_lid in segment(waypoints):
                stamp = runner.get_clock().now().to_msg()
                poses = [_pose(w['x'], w['y'], w.get('yaw', 0.0), stamp) for w in seg]

                runner.get_logger().info(f'Following {len(poses)} waypoint(s)')
                nav.followWaypoints(poses)

                while not nav.isTaskComplete():
                    fb = nav.getFeedback()
                    if fb is not None:
                        runner.get_logger().info(
                            f'  at waypoint {fb.current_waypoint + 1}/{len(poses)}',
                            throttle_duration_sec=2.0)

                result = nav.getResult()
                if result != TaskResult.SUCCEEDED:
                    runner.get_logger().error(f'Segment ended: {result}')
                    return

                if ends_with_lid:
                    runner.cycle_lid(lid_dwell)

            if not loop:
                runner.get_logger().info('Route complete')
                return
            runner.get_logger().info('Route complete — looping')

    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
