#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge


class ColorTracker:
    """
    Constant-velocity Kalman filter for a 3D object position.
    State: [x, y, z, vx, vy, vz]. Measurement: [x, y, z].
    Publishes only after min_frames stable detections — same principle used
    in VIO pipelines to avoid reporting spurious tracks.
    """

    def __init__(self, initial_pos, dt=0.1, min_frames=5):
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0,  0 ],
            [0, 1, 0, 0,  dt, 0 ],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.eye(3, 6, dtype=np.float32)
        self.kf.processNoiseCov     = np.eye(6, dtype=np.float32) * 1e-4
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-3
        self.kf.errorCovPost = np.eye(6, dtype=np.float32)
        self.kf.statePost = np.array(
            [[initial_pos[0]], [initial_pos[1]], [initial_pos[2]], [0], [0], [0]],
            dtype=np.float32,
        )
        self.frames_seen = 0
        self.min_frames = min_frames

    def update(self, pos):
        """Fuse new measurement. Returns True once the filter has converged."""
        meas = np.array([[pos[0]], [pos[1]], [pos[2]]], dtype=np.float32)
        self.kf.predict()
        self.kf.correct(meas)
        self.frames_seen += 1
        return self.frames_seen >= self.min_frames

    def get_position(self):
        s = self.kf.statePost
        return [float(s[0]), float(s[1]), float(s[2])]


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)

        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/workspace/object_map', 10)

        self.bridge = CvBridge()
        self.latest_depth = None   # float32 numpy array, metres per pixel

        # Camera intrinsics — derived from SDF: width=640, height=320, hfov=1.0 rad
        self.fx = 585.0
        self.fy = 588.0
        self.cx = 320.0
        self.cy = 160.0

        # Camera pose in panda_link0 frame
        self.cam_x = 0.6
        self.cam_y = 0.6
        self.cam_z = 1.1

        # Fallback depth/z when depth image is not yet available
        self.block_z = 0.70
        self.depth    = self.cam_z - self.block_z   # 0.40 m

        # Kalman filter trackers and persistent object map
        self.trackers   = {}   # color_id -> ColorTracker
        self.object_map = {}   # color_id -> [x, y, z]

        # Per-color marker colours (R, G, B, A)
        self.COLOR_RGBA = {
            "R": (1.0, 0.1, 0.1, 0.85),
            "G": (0.1, 0.85, 0.1, 0.85),
            "B": (0.1, 0.3,  1.0, 0.85),
        }

        self._frames_received = 0
        self.get_logger().info(
            f"Color Detector started — camera at ({self.cam_x}, {self.cam_y}, {self.cam_z}). "
            f"Depth: sensor if available, else fixed {self.depth:.2f}m (blocks at Z={self.block_z})."
        )

    # ------------------------------------------------------------------
    # Depth callback — keeps the most recent depth frame for 3-D lookup
    # ------------------------------------------------------------------
    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    # ------------------------------------------------------------------
    # RGB callback — detect colours, project to 3-D, Kalman-filter, publish
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        self._frames_received += 1
        if self._frames_received % 30 == 1:
            self.get_logger().info(f"Frames received: {self._frames_received}")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "R": [(0, 80, 80),   (15, 255, 255)],
            "G": [(45, 80, 80),  (90, 255, 255)],
            "B": [(100, 80, 80), (140, 255, 255)],
        }
        red_mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))

        for color_id, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            if color_id == "R":
                mask = cv2.bitwise_or(mask, red_mask2)
            mask = cv2.erode(mask,  None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500 or area > 30000:   # skip noise AND large table-surface false positives
                    continue

                bx, by, bw, bh = cv2.boundingRect(cnt)
                u = bx + bw // 2   # pixel column (horizontal)
                v = by + bh // 2   # pixel row    (vertical)

                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 255), 2)
                cv2.putText(frame, color_id, (bx, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # --- 3-D projection using camera-adaptive depth ---
                # Camera is pitched π/2 downward (sensors.xacro pose 0 0 0 0 1.57 0)
                # image-up  (v decreases) → panda +X
                # image-right(u increases) → panda -Y
                depth, z_world, depth_src = self._get_depth(v, u)
                X_world = self.cam_x - (v - self.cy) * depth / self.fy
                Y_world = self.cam_y - (u - self.cx) * depth / self.fx
                Z_world = z_world

                # --- Kalman filter update ---
                raw_pos = [X_world, Y_world, Z_world]
                if color_id not in self.trackers:
                    self.trackers[color_id] = ColorTracker(raw_pos, min_frames=3)
                    converged = False
                else:
                    converged = self.trackers[color_id].update(raw_pos)

                if converged:
                    px, py, pz = self.trackers[color_id].get_position()
                    out = f"{color_id},{px:.3f},{py:.3f},{pz:.3f}"
                    self.coords_pub.publish(String(data=out))
                    self.get_logger().info(
                        f"[KF/{depth_src}] {out}  d={depth:.3f}m"
                    )

                    # Update live workspace object map
                    self.object_map[color_id] = [px, py, pz]
                    self._publish_object_map()

        try:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 320)
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"OpenCV display error: {e}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _get_depth(self, v, u):
        """Return (depth_metres, z_world, source_tag).

        Uses depth sensor if available; otherwise falls back to the known
        constant depth (blocks placed at Z=0.70, camera at Z=1.1 → depth=0.40m).
        Size-based monocular estimation is intentionally omitted: bounding-box
        pixel-width noise feeds directly into the XY projection formulas (not
        just Z), so a 10% depth error becomes a ~15 cm XY positioning error.
        """
        # Real depth sensor
        if self.latest_depth is not None:
            try:
                d = float(self.latest_depth[v, u])
                if np.isfinite(d) and 0.05 < d < 0.55:
                    return d, self.cam_z - d, 'sensor'
            except IndexError:
                pass

        # Known constant — blocks are at Z=0.70, camera at Z=1.1
        return self.depth, self.block_z, 'fixed'

    def _publish_object_map(self):
        """Publish all known object positions as coloured 3-D cubes in RViz."""
        markers = MarkerArray()
        for i, (cid, pos) in enumerate(self.object_map.items()):
            m = Marker()
            m.header.frame_id = "panda_link0"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "workspace_objects"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.scale.x = 0.06
            m.scale.y = 0.06
            m.scale.z = 0.10
            r, g, b, a = self.COLOR_RGBA.get(cid, (1.0, 1.0, 1.0, 0.8))
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            m.lifetime = Duration(sec=1)   # auto-expire if object stops being detected
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
