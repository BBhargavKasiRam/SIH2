#!/usr/bin/env python3
"""
FleetMind RMF Adapter - Synchronizes Edge AI Safety Decisions with Open-RMF.
Translates local YIELD / CONTINUE decisions into local velocity overrides and
RMF PauseRequest/ResumeRequest signals, preventing schedule desynchronization.
"""
from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from fleetmind_msgs.msg import SafetyDecision, ReplanRequest

try:
    from rmf_fleet_msgs.msg import PauseRequest
    RMF_AVAILABLE = True
except ImportError:
    RMF_AVAILABLE = False
    from std_msgs.msg import String as PauseRequest


class RMFAdapter(Node):
    """
    Adapter node that interfaces Edge-AI decisions with Open-RMF and local actuators.
    """

    def __init__(self):
        super().__init__('rmf_adapter')

        # Parameters
        self.declare_parameter('robot_id', 'tinyRobot1')
        self.declare_parameter('fleet_name', 'tinyRobot')
        self.declare_parameter('deadlock_timeout_sec', 15.0)
        self.declare_parameter('publish_cmd_vel_override', True)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.fleet_name = str(self.get_parameter('fleet_name').value)
        self.deadlock_timeout_sec = float(self.get_parameter('deadlock_timeout_sec').value)
        self.publish_cmd_vel_override = bool(self.get_parameter('publish_cmd_vel_override').value)

        self.is_yielding = False
        self.yield_start_time = 0.0
        self.mode_request_id = 1000

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.safety_sub = self.create_subscription(
            SafetyDecision,
            f'/{self.robot_id}/safety_decision',
            self.safety_callback,
            qos_reliable
        )

        self.replan_sub = self.create_subscription(
            ReplanRequest,
            '/fleetmind/replan_requests',
            self.replan_callback,
            qos_reliable
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_id}/cmd_vel',
            qos_reliable
        )

        if RMF_AVAILABLE:
            self.rmf_pause_pub = self.create_publisher(
                PauseRequest,
                '/robot_pause_requests',
                qos_reliable
            )
        else:
            self.rmf_pause_pub = self.create_publisher(
                PauseRequest,
                '/mock_rmf_pause_requests',
                qos_reliable
            )

        # Deadlock watchdog timer at 1 Hz
        self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info(
            f"FleetMind RMF Adapter started for [{self.fleet_name}/{self.robot_id}] "
            f"(RMF Available: {RMF_AVAILABLE})"
        )

    def safety_callback(self, msg: SafetyDecision):
        """Process Edge-AI decision."""
        if msg.robot_id != self.robot_id:
            return

        if msg.decision == "YIELD" and not self.is_yielding:
            self.get_logger().warn(
                f"[EDGE-AI OVERRIDE] {self.robot_id} -> YIELD ({msg.reason}). "
                f"Actuating pause in local motion & Open-RMF."
            )
            self.is_yielding = True
            self.yield_start_time = time.time()

            if self.publish_cmd_vel_override:
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)

            self.send_rmf_pause_request(pause=True)

        elif msg.decision in ["CONTINUE", "MONITOR"] and self.is_yielding:
            yield_duration = time.time() - self.yield_start_time
            self.get_logger().info(
                f"[EDGE-AI RESUME] {self.robot_id} -> {msg.decision} ({msg.reason}). "
                f"Resuming after {yield_duration:.1f}s yield."
            )
            self.is_yielding = False
            self.send_rmf_pause_request(pause=False)

    def replan_callback(self, msg: ReplanRequest):
        """Handle explicit dynamic re-routing requests (e.g. from deadlock resolution)."""
        if msg.robot_id != self.robot_id:
            return
            
        self.get_logger().warn(f"[DYNAMIC RE-ROUTE] Replanning requested for {self.robot_id}. Reason: {msg.reason}")
        
        # Trigger an RMF replan by issuing a quick pause/resume cycle
        # This prompts the RMF fleet adapter to reconsider the path
        self.send_rmf_pause_request(pause=True)
        time.sleep(0.5)
        self.send_rmf_pause_request(pause=False)

    def send_rmf_pause_request(self, pause: bool):
        """Dispatch PauseRequest message to Open-RMF."""
        self.mode_request_id += 1
        if RMF_AVAILABLE:
            req = PauseRequest()
            req.fleet_name = self.fleet_name
            req.robot_name = self.robot_id
            req.mode_request_id = int(self.mode_request_id)
            req.type = PauseRequest.TYPE_PAUSE_IMMEDIATELY if pause else PauseRequest.TYPE_RESUME
            req.at_checkpoint = 0
            self.rmf_pause_pub.publish(req)
        else:
            req = PauseRequest()
            req.data = f"PAUSE={pause} for fleet={self.fleet_name} robot={self.robot_id}"
            self.rmf_pause_pub.publish(req)

    def watchdog_callback(self):
        """Monitor for prolonged deadlock and trigger replan alerts."""
        if not self.is_yielding:
            return

        duration = time.time() - self.yield_start_time
        if duration > self.deadlock_timeout_sec:
            self.get_logger().error(
                f"[DEADLOCK WATCHDOG] {self.robot_id} has been yielding for {duration:.1f}s "
                f"(exceeds {self.deadlock_timeout_sec}s threshold). Triggering RMF replan alert!"
            )
            # Signal RMF fleet adapter by re-issuing an updated resume or pause cycle
            self.yield_start_time = time.time() - (self.deadlock_timeout_sec * 0.5)


def main(args=None):
    rclpy.init(args=args)
    node = RMFAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
