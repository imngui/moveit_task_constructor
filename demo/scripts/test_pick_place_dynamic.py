#!/usr/bin/env python3
"""
Round-trip test for pick_place_dynamic_demo.

Sends two PickPlaceTask messages in sequence:
  1. Pick the object from its spawn position and place it at the "away" pose.
  2. Pick it back and return it to the original spawn position.

The script waits for each task to finish by monitoring the /statistics topic:
it blocks until the root stage (stage id=1) reports at least one solution, then
waits an extra `execution_timeout_s` seconds to cover actual execution time before
sending the next request.

Prerequisites:
  - MoveIt, controllers, and robot_state_publisher running
  - pick_place_dynamic_demo node running:
      ros2 run moveit_task_constructor_demo pick_place_dynamic_demo \
        --ros-args --params-file <path_to>/panda_config.yaml

Usage:
  python3 test_pick_place_dynamic.py
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from moveit_task_constructor_msgs.msg import PickPlaceTask, TaskStatistics

# ── Pose configuration (must match panda_config.yaml) ────────────────────────
FRAME_ID = "world"

# Where the object starts (matches object_pose in panda_config.yaml)
OBJECT_SPAWN = dict(x=0.5, y=-0.25, z=0.0)

# Where to place the object on the first move (matches place_pose in panda_config.yaml)
PLACE_AWAY = dict(x=0.6, y=-0.15, z=0.0)

OBJECT_ID = "object"

# How long to wait after a solution is found before assuming execution is done.
# Increase this if your robot moves slowly.
EXECUTION_TIMEOUT_S = 45.0

# ─────────────────────────────────────────────────────────────────────────────


def make_pose(x: float, y: float, z: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = FRAME_ID
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.w = 1.0
    return p


class RoundTripTest(Node):
    def __init__(self):
        super().__init__("pick_place_round_trip_test")
        self._pub = self.create_publisher(PickPlaceTask, "/pick_place_task", 10)

        transient_local_qos = QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._stats_sub = self.create_subscription(
            TaskStatistics,
            "/statistics",
            self._stats_cb,
            transient_local_qos,
        )

        self._solution_found = threading.Event()

    def _stats_cb(self, msg: TaskStatistics):
        # Stage id=1 is the root SerialContainer; solutions there mean a full
        # end-to-end solution was found.
        for stage in msg.stages:
            if stage.id == 1 and len(stage.solved) > 0:
                self._solution_found.set()
                return

    def _wait_for_solution(self, timeout: float = 120.0) -> bool:
        """Block until a solution appears in stage 1, or timeout."""
        self._solution_found.clear()
        self.get_logger().info("  Waiting for planning to produce a solution …")
        found = self._solution_found.wait(timeout=timeout)
        if not found:
            self.get_logger().error("  Timed out waiting for a solution!")
        return found

    def send(self, object_id: str, place: dict, label: str):
        msg = PickPlaceTask()
        msg.object_id = object_id
        msg.place_pose = make_pose(**place)
        self.get_logger().info(
            f"[{label}] Sending task: pick '{object_id}' → "
            f"place at ({place['x']:.3f}, {place['y']:.3f}, {place['z']:.3f})"
        )
        self._pub.publish(msg)

    def run(self):
        # ── Move 1: pick from spawn → place away ─────────────────────────────
        self.send(OBJECT_ID, PLACE_AWAY, "move 1/2")
        if not self._wait_for_solution():
            return False

        self.get_logger().info(
            f"  Solution found. Waiting {EXECUTION_TIMEOUT_S}s for execution to complete …"
        )
        time.sleep(EXECUTION_TIMEOUT_S)

        # ── Move 2: pick from away → return to spawn ─────────────────────────
        self.send(OBJECT_ID, OBJECT_SPAWN, "move 2/2")
        if not self._wait_for_solution():
            return False

        self.get_logger().info(
            f"  Solution found. Waiting {EXECUTION_TIMEOUT_S}s for execution to complete …"
        )
        time.sleep(EXECUTION_TIMEOUT_S)

        self.get_logger().info("Round-trip complete.")
        return True


def main():
    rclpy.init()
    node = RoundTripTest()

    # Spin in a background thread so callbacks fire while main thread sleeps
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    success = node.run()
    rclpy.shutdown()
    spin_thread.join(timeout=5.0)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
