"""Cluster 2D lidar scan points into obstacles and publish the result."""

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from obstacle_detector_msgs.msg import Obstacle, ObstacleArray


class ObstacleDetector(Node):
    """Detect obstacles by segmenting LaserScan points with a distance gap."""

    MAX_INDEX_GAP = 2

    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('output_frame', 'laser_link')
        self.declare_parameter('cluster_distance', 0.30)
        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('warning_distance', 1.0)
        self.declare_parameter('marker_lifetime', 0.5)

        self.output_frame = self.get_parameter('output_frame').value
        self.cluster_distance = float(self.get_parameter('cluster_distance').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.warning_distance = float(self.get_parameter('warning_distance').value)
        self.marker_lifetime = float(self.get_parameter('marker_lifetime').value)
        self.marker_lifetime_msg = Duration(
            sec=int(self.marker_lifetime),
            nanosec=int(round((self.marker_lifetime % 1.0) * 1e9)))

        self.obstacle_pub = self.create_publisher(ObstacleArray, '/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles/markers', 10)
        self.nearest_pub = self.create_publisher(
            Float32, '/nearest_obstacle_distance', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            'Obstacle detector ready: cluster_distance=%.2f m, '
            'min_cluster_points=%d' % (self.cluster_distance, self.min_cluster_points))

    # ------------------------------------------------------------------ #
    # Scan processing
    # ------------------------------------------------------------------ #
    def _scan_to_points(self, scan):
        """Return (index, angle, x, y, range) tuples for valid scan returns."""
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < self.min_range or distance > self.max_range:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            points.append((
                index,
                angle,
                distance * math.cos(angle),
                distance * math.sin(angle),
                distance,
            ))
        return points

    def _cluster_points(self, points):
        """Split ordered scan points wherever the gap between them is large."""
        clusters = []
        current = []
        previous = None

        for point in points:
            if previous is not None:
                index_jump = point[0] - previous[0]
                distance_jump = math.hypot(
                    point[2] - previous[2], point[3] - previous[3])
                if (index_jump > self.MAX_INDEX_GAP
                        or distance_jump > self.cluster_distance):
                    if len(current) >= self.min_cluster_points:
                        clusters.append(current)
                    current = []

            current.append(point)
            previous = point

        if len(current) >= self.min_cluster_points:
            clusters.append(current)

        return clusters

    def _build_obstacle(self, cluster, obstacle_id, stamp):
        count = len(cluster)
        xs = [point[2] for point in cluster]
        ys = [point[3] for point in cluster]
        centroid_x = sum(xs) / count
        centroid_y = sum(ys) / count

        obstacle = Obstacle()
        obstacle.header.stamp = stamp
        obstacle.header.frame_id = self.output_frame
        obstacle.id = obstacle_id
        obstacle.position = Point(x=centroid_x, y=centroid_y, z=0.0)
        obstacle.distance = float(math.hypot(centroid_x, centroid_y))
        obstacle.angle = float(math.atan2(centroid_y, centroid_x))
        obstacle.width = float(math.hypot(
            max(xs) - min(xs), max(ys) - min(ys)))
        obstacle.point_count = count
        return obstacle

    # ------------------------------------------------------------------ #
    # Visualisation
    # ------------------------------------------------------------------ #
    def _build_markers(self, obstacles, stamp):
        markers = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self.output_frame
        clear.header.stamp = stamp
        clear.ns = 'obstacles'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for obstacle in obstacles:
            diameter = max(0.15, obstacle.width)
            ratio = min(max(obstacle.distance / max(self.max_range, 1e-6), 0.0), 1.0)

            cylinder = Marker()
            cylinder.header.frame_id = self.output_frame
            cylinder.header.stamp = stamp
            cylinder.ns = 'obstacles'
            cylinder.id = obstacle.id
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position = obstacle.position
            cylinder.pose.position.z = 0.20
            cylinder.pose.orientation.w = 1.0
            cylinder.scale.x = diameter
            cylinder.scale.y = diameter
            cylinder.scale.z = 0.30
            cylinder.color.r = 1.0 - ratio
            cylinder.color.g = ratio
            cylinder.color.b = 0.10
            cylinder.color.a = 0.75
            cylinder.lifetime = self.marker_lifetime_msg
            markers.markers.append(cylinder)

            label = Marker()
            label.header.frame_id = self.output_frame
            label.header.stamp = stamp
            label.ns = 'obstacles'
            label.id = 1000 + obstacle.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = obstacle.position.x
            label.pose.position.y = obstacle.position.y
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.lifetime = self.marker_lifetime_msg
            label.text = '#%d %.2fm' % (obstacle.id, obstacle.distance)
            markers.markers.append(label)

        return markers

    # ------------------------------------------------------------------ #
    # Callback
    # ------------------------------------------------------------------ #
    def scan_callback(self, scan):
        points = self._scan_to_points(scan)
        clusters = self._cluster_points(points)
        obstacles = [
            self._build_obstacle(cluster, obstacle_id, scan.header.stamp)
            for obstacle_id, cluster in enumerate(clusters)
        ]

        array = ObstacleArray()
        array.header.stamp = scan.header.stamp
        array.header.frame_id = self.output_frame
        array.obstacles = obstacles
        self.obstacle_pub.publish(array)
        self.marker_pub.publish(self._build_markers(obstacles, scan.header.stamp))

        nearest = min(
            (obstacle.distance for obstacle in obstacles), default=None)
        if nearest is None:
            self.nearest_pub.publish(Float32(data=-1.0))
            return

        self.nearest_pub.publish(Float32(data=float(nearest)))
        if nearest < self.warning_distance:
            self.get_logger().warn(
                'Obstacle at %.2f m (%d detected)' % (nearest, len(obstacles)),
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
