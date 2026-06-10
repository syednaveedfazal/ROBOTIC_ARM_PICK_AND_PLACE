#!/usr/bin/env python3
"""
Sequential multi-color pick-and-place.

Trigger: first /color_coordinates message starts a 2-second timer so all
three blocks (now all in camera FOV) can be detected before the arm moves.
The timer callback runs inside the ROS2 executor — this is the proven-working
pattern for pymoveit2 (avoids "generator already executing" seen with raw Threads).

Z-axis strategy: block centre Z is KNOWN from the world file (0.70 m).
Camera is used for XY detection only.  Camera-estimated Z is unreliable
because bounding-box pixel-width noise feeds directly into the depth formula
and can push the grasp target below the table surface — causing MoveIt to
refuse the plan entirely.
"""

import math
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

SORT_ORDER = ["R", "G", "B"]

# Known block geometry (from empty.world)
BLOCK_CENTER_Z   = 0.70   # block link pose Z
FINGERTIP_OFFSET = 0.103  # panda_hand → fingertip contact surface (m)
APPROACH_OFFSET  = 0.20   # hover clearance above grasp Z (m)

# Derived grasp heights (fixed, reliable)
EEF_GRASP_Z = BLOCK_CENTER_Z + FINGERTIP_OFFSET   # 0.803 m
HOVER_Z     = EEF_GRASP_Z + APPROACH_OFFSET        # 1.003 m


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        # Higher velocity/acceleration for faster, snappier motion
        self.moveit2.max_velocity     = 0.7
        self.moveit2.max_acceleration = 0.5

        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Joint configurations
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [
            0.0, 0.0, 0.0,
            math.radians(-90.0), 0.0,
            math.radians(92.0), math.radians(50.0),
        ]
        _drop_rest = [
            math.radians(30.0), math.radians(-20.0),
            math.radians(-124.0), math.radians(44.0),
            math.radians(163.0), math.radians(7.0),
        ]
        self.drop_joints = {
            "R": [math.radians(-130.0)] + _drop_rest,
            "G": [math.radians(-155.0)] + _drop_rest,
            "B": [math.radians(-175.0)] + _drop_rest,
        }

        self._lock           = Lock()
        self.detected_blocks = {}    # color_id → [x, y, z]
        self.sorted_colors   = set()
        self._sort_timer     = None  # created on first detection

        self.sub = self.create_subscription(
            String, "/color_coordinates", self._coords_cb, 10,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"PickAndPlace ready — grasp_z={EEF_GRASP_Z:.3f}  hover_z={HOVER_Z:.3f}"
        )

        # Safe start position (called before executor starts — pymoveit2 handles this)
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

    # ------------------------------------------------------------------ #
    def _coords_cb(self, msg):
        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()
        except ValueError:
            return

        with self._lock:
            if color_id not in self.sorted_colors:
                self.detected_blocks[color_id] = [float(x), float(y), float(z)]

            # Fire the sort timer once, 2 s after the first detection
            if self._sort_timer is None:
                self._sort_timer = self.create_timer(
                    2.0, self._trigger_sort,
                    callback_group=self.callback_group,
                )
                self.get_logger().info(
                    f"First block seen ({color_id}) — sort starts in 2 s …"
                )

    # ------------------------------------------------------------------ #
    def _trigger_sort(self):
        """Timer callback — runs inside the executor (pymoveit2-safe)."""
        self._sort_timer.cancel()

        with self._lock:
            snapshot = {c: list(v) for c, v in self.detected_blocks.items()}

        self.get_logger().info(f"=== Sort start — detected: {list(snapshot.keys())} ===")

        for color_id in SORT_ORDER:
            coords = snapshot.get(color_id)
            if coords is None:
                self.get_logger().info(f"  {color_id}: not detected — skipping.")
                continue
            self.get_logger().info(f"  Picking {color_id} at XY=({coords[0]:.3f},{coords[1]:.3f})")
            self._pick_and_place_one(color_id, coords)
            with self._lock:
                self.sorted_colors.add(color_id)

        self.get_logger().info("=== All done — returning home. ===")
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()
        rclpy.shutdown()

    # ------------------------------------------------------------------ #
    def _pick_and_place_one(self, color_id, coords):
        bx, by = coords[0], coords[1]   # XY from camera
        quat   = [0.0, 1.0, 0.0, 0.0]  # gripper pointing down

        # Grasp and hover heights are FIXED (known from world file).
        # Camera Z is intentionally ignored here — pixel-width noise in the
        # depth formula can shift the target below the table, causing MoveIt
        # to reject the plan silently and the arm to never descend.
        hover = [bx, by, HOVER_Z]
        grasp = [bx, by, EEF_GRASP_Z]

        self.get_logger().info(
            f"    [{color_id}] XY=({bx:.3f},{by:.3f})  "
            f"hover_z={HOVER_Z:.3f}  grasp_z={EEF_GRASP_Z:.3f}"
        )

        # 1. Home
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above block
        self.moveit2.move_to_pose(position=hover, quat_xyzw=quat)
        self.moveit2.wait_until_executed()

        # 3. Open gripper
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 4. Descend — Cartesian straight-line (all blocks at X=0.6, pure Z drop)
        self.moveit2.move_to_pose(position=grasp, quat_xyzw=quat, cartesian=True)
        self.moveit2.wait_until_executed()

        # 5. Close gripper and wait for physics to register contact
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 6. Home (MoveIt joint-space plan lifts arm before swinging to drop)
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 7. Rotate to colour bin
        self.moveit2.move_to_configuration(self.drop_joints[color_id])
        self.moveit2.wait_until_executed()

        # 8. Release
        self.gripper.open()
        self.gripper.wait_until_executed()
        self.gripper.close()
        self.gripper.wait_until_executed()

        self.get_logger().info(f"    {color_id} deposited.")


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    t = Thread(target=executor.spin, daemon=True)
    t.start()

    try:
        t.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
