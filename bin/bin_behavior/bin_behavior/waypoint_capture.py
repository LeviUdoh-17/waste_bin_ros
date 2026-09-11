"""Record waypoints by driving the robot and capturing where it actually is.

Teleop to a spot, call the capture service, and the robot's current
map -> base_footprint transform is appended to waypoints.yaml.

Why TF and not RViz clicks: slam.rviz's "2D Goal Pose" tool publishes to `goal_pose`,
which is the topic Nav2's bt_navigator consumes, so capturing from it would send the
robot driving to every point as you clicked. `/clicked_point` is unused by Nav2 but
carries no orientation. Reading the transform sidesteps both, and records a pose the
robot has demonstrably reached — a yaw it can achieve, at a spot that is not buried
inside a costmap inflation zone.

Works during either run mode: slam_toolbox (mapping) and AMCL (navigation) both
publish map -> odom, so a route can be recorded on the same drive that builds the map.

    ros2 run bin_behavior waypoint_capture
    ros2 service call /waypoint_capture/capture std_srvs/srv/SetBool "{data: true}"
    ros2 service call /waypoint_capture/undo    std_srvs/srv/Trigger
    ros2 service call /waypoint_capture/clear   std_srvs/srv/Trigger
"""

import math
import os

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

HEADER = """\
# Waypoints for bin_behavior/waypoint_runner, in the map frame.
#
# Captured with bin_behavior/waypoint_capture: drive the robot to each spot and call
#   ros2 service call /waypoint_capture/capture std_srvs/srv/SetBool "{data: true}"
# data: true marks the waypoint open_lid, so the route pauses and cycles the lid there.
#
# yaw is radians, CCW from +x. Hand-editing is fine; keep the same keys.

"""


class WaypointCapture(Node):
    def __init__(self):
        super().__init__('waypoint_capture')

        default_file = os.path.join(
            get_package_share_directory('bin_bringup'), 'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_file', default_file)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.path = self.get_parameter('waypoints_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.create_service(SetBool, '~/capture', self._capture_cb)
        self.create_service(Trigger, '~/undo', self._undo_cb)
        self.create_service(Trigger, '~/clear', self._clear_cb)

        cfg = self._load()
        self.get_logger().info(
            f'waypoint_capture ready — {self.path} '
            f'({len(cfg["waypoints"])} waypoint(s) already recorded)')
        self.get_logger().info(
            f'capturing {self.map_frame} -> {self.base_frame}')

    # ── file ─────────────────────────────────────────────────────────────

    def _load(self):
        try:
            with open(self.path) as f:
                cfg = yaml.safe_load(f) or {}
        except FileNotFoundError:
            cfg = {}
        cfg.setdefault('waypoints', [])
        cfg.setdefault('loop', False)
        cfg.setdefault('lid_dwell', 3.0)
        return cfg

    def _save(self, cfg):
        # yaml.safe_dump drops comments, so the header is re-emitted rather than
        # round-tripped. default_flow_style=None keeps each waypoint on one line.
        body = yaml.safe_dump(cfg, default_flow_style=None, sort_keys=False)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(HEADER)
            f.write(body)
        os.replace(tmp, self.path)

    # ── pose ─────────────────────────────────────────────────────────────

    def _current_pose(self):
        """(x, y, yaw) of base_frame in map_frame, or None with a reason logged."""
        try:
            tf = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'no {self.map_frame} -> {self.base_frame}: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # Planar case: roll and pitch are zero, so yaw reduces to this.
        yaw = 2.0 * math.atan2(q.z, q.w)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))   # wrap to [-pi, pi]
        return t.x, t.y, yaw

    # ── services ─────────────────────────────────────────────────────────

    def _capture_cb(self, request, response):
        pose = self._current_pose()
        if pose is None:
            response.success = False
            response.message = (
                f'No {self.map_frame} -> {self.base_frame} transform. Is SLAM or AMCL '
                f'running, and has AMCL been given an initial pose?')
            return response

        x, y, yaw = pose
        cfg = self._load()
        cfg['waypoints'].append({
            'x': round(float(x), 3),
            'y': round(float(y), 3),
            'yaw': round(float(yaw), 3),
            'open_lid': bool(request.data),
        })
        self._save(cfg)

        n = len(cfg['waypoints'])
        response.success = True
        response.message = (
            f'waypoint {n}: x={x:.3f} y={y:.3f} yaw={yaw:.3f} '
            f'open_lid={bool(request.data)}')
        self.get_logger().info(response.message)
        return response

    def _undo_cb(self, request, response):
        cfg = self._load()
        if not cfg['waypoints']:
            response.success = False
            response.message = 'Nothing to undo — route is already empty'
            return response
        dropped = cfg['waypoints'].pop()
        self._save(cfg)
        response.success = True
        response.message = (
            f'Removed waypoint at x={dropped["x"]} y={dropped["y"]}, '
            f'{len(cfg["waypoints"])} left')
        self.get_logger().info(response.message)
        return response

    def _clear_cb(self, request, response):
        cfg = self._load()
        n = len(cfg['waypoints'])
        cfg['waypoints'] = []
        self._save(cfg)
        response.success = True
        response.message = f'Cleared {n} waypoint(s)'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCapture()
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
