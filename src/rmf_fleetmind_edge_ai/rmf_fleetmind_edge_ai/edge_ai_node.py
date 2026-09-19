#!/usr/bin/env python3
"""
FleetMind Edge AI Node - Distributed Onboard Intelligence for AMRs.
Runs locally per robot, predicts collision risks over DDS P2P intent,
and negotiates right-of-way without central authority.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from rmf_fleet_msgs.msg import FleetState
from fleetmind_msgs.msg import PeerIntent, SafetyDecision, Heartbeat
from fleetmind_msgs.msg import RobotState as FleetMindRobotState
from geometry_msgs.msg import Point

from rmf_fleetmind_edge_ai.collision_predictor import (
    MLCollisionPredictor,
    DEFAULT_ROBOT_RADIUS,
    DEFAULT_SAFETY_MARGIN,
)


class EdgeAINode(Node):
    """
    Onboard Edge-AI node running for a single AMR.
    """

    def __init__(self):
        super().__init__('edge_ai_node')

        # Parameters
        self.declare_parameter('robot_id', 'tinyRobot1')
        self.declare_parameter('fleet_name', 'tinyRobot')
        self.declare_parameter('robot_radius', DEFAULT_ROBOT_RADIUS)
        self.declare_parameter('safety_margin', DEFAULT_SAFETY_MARGIN)
        self.declare_parameter('lookahead_horizon', 5.0)
        self.declare_parameter('prediction_rate_hz', 10.0)
        self.declare_parameter('broadcast_rate_hz', 10.0)
        self.declare_parameter('yield_lockin_sec', 3.0)
        self.declare_parameter('stale_peer_sec', 1.0)
        self.declare_parameter('lost_peer_sec', 10.0)
        self.declare_parameter('task_priority', 1)
        self.declare_parameter('ml_model_path', '')

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.fleet_name = str(self.get_parameter('fleet_name').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.lookahead_horizon = float(self.get_parameter('lookahead_horizon').value)
        self.yield_lockin_sec = float(self.get_parameter('yield_lockin_sec').value)
        self.stale_peer_sec = float(self.get_parameter('stale_peer_sec').value)
        self.lost_peer_sec = float(self.get_parameter('lost_peer_sec').value)
        self.task_priority = int(self.get_parameter('task_priority').value)

        model_path = str(self.get_parameter('ml_model_path').value)
        if not model_path:
            # Default to installed model asset if present
            default_pkg_model = os.path.join(
                os.path.dirname(__file__), '..', 'models', 'best_model.joblib'
            )
            if os.path.exists(default_pkg_model):
                model_path = default_pkg_model

        self.predictor = MLCollisionPredictor(model_path=model_path if model_path else None)
        if self.predictor.ml_loaded:
            self.get_logger().info(f"Loaded Edge-AI ML model from {model_path}")
        else:
            self.get_logger().info("Running in deterministic kinematics Edge-AI mode.")

        # Local robot state
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.battery_percent = 100.0
        self.current_task_id = ""
        self.last_state_time = 0.0

        # Motion intent & decision state
        self.current_intent = "MOVING"
        self.current_target_peer = ""
        self.yield_lock_target: Optional[str] = None
        self.yield_lock_time = 0.0
        self.yield_start_time = 0.0

        # Peer knowledge: peer_id -> dict
        self.peers: Dict[str, Dict[str, Any]] = {}
        self.recently_missing_peers: Dict[str, float] = {}

        # QoS Profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self.odom_callback,
            qos_best_effort
        )

        self.fleet_state_sub = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_state_callback,
            qos_best_effort
        )

        self.peer_intent_sub = self.create_subscription(
            PeerIntent,
            '/fleetmind/p2p_intent',
            self.peer_intent_callback,
            qos_reliable
        )

        # Publishers
        self.intent_pub = self.create_publisher(
            PeerIntent,
            '/fleetmind/p2p_intent',
            qos_reliable
        )

        self.safety_pub = self.create_publisher(
            SafetyDecision,
            f'/{self.robot_id}/safety_decision',
            qos_reliable
        )

        self.global_safety_pub = self.create_publisher(
            SafetyDecision,
            '/fleetmind/safety_decision',
            qos_reliable
        )

        self.heartbeat_pub = self.create_publisher(
            Heartbeat,
            '/fleetmind/heartbeats',
            qos_reliable
        )

        self.robot_state_pub = self.create_publisher(
            FleetMindRobotState,
            '/fleetmind/robot_states',
            qos_reliable
        )

        # Timers
        broadcast_period = 1.0 / max(float(self.get_parameter('broadcast_rate_hz').value), 1.0)
        prediction_period = 1.0 / max(float(self.get_parameter('prediction_rate_hz').value), 1.0)

        self.create_timer(broadcast_period, self.broadcast_timer_callback)
        self.create_timer(prediction_period, self.prediction_timer_callback)
        self.create_timer(1.0, self.heartbeat_timer_callback)

        self.get_logger().info(f"FleetMind Edge AI Node started for [{self.fleet_name}/{self.robot_id}]")

    def odom_callback(self, msg: Odometry):
        """Update local state from direct robot odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.last_state_time = time.time()

    def fleet_state_callback(self, msg: FleetState):
        """Fallback to update state from RMF /fleet_states if direct odom is unmapped."""
        now = time.time()
        for robot in msg.robots:
            if robot.name == self.robot_id:
                # Update velocity from position delta if odom hasn't reported recently
                if now - self.last_state_time > 0.5:
                    dt = max(now - self.last_state_time, 0.01) if self.last_state_time > 0 else 0.1
                    self.vx = (robot.location.x - self.x) / dt
                    self.vy = (robot.location.y - self.y) / dt
                    self.x = robot.location.x
                    self.y = robot.location.y
                    self.yaw = robot.location.yaw
                    self.last_state_time = now

                self.battery_percent = robot.battery_percent
                self.current_task_id = robot.task_id

    def peer_intent_callback(self, msg: PeerIntent):
        """Record and update peer AMR intent received over DDS."""
        if msg.robot_id == self.robot_id:
            return  # Disregard own broadcasts

        self.peers[msg.robot_id] = {
            'fleet_name': msg.fleet_name,
            'x': msg.position.position.x,
            'y': msg.position.position.y,
            'vx': msg.velocity.linear.x,
            'vy': msg.velocity.linear.y,
            'intent': msg.intent,
            'target_peer': msg.target_peer,
            'priority': msg.priority,
            'wait_time_sec': msg.wait_time_sec,
            'last_seen': time.time()
        }

    def broadcast_timer_callback(self):
        """Publish local motion intent to neighboring peer AMRs."""
        wait_duration = (time.time() - self.yield_start_time) if self.current_intent == "YIELDING" else 0.0

        msg = PeerIntent()
        msg.robot_id = self.robot_id
        msg.fleet_name = self.fleet_name
        msg.timestamp = self.get_clock().now().to_msg()
        msg.position.position.x = self.x
        msg.position.position.y = self.y
        msg.velocity.linear.x = self.vx
        msg.velocity.linear.y = self.vy
        msg.intent = self.current_intent
        msg.target_peer = self.current_target_peer
        msg.priority = self.task_priority
        msg.wait_time_sec = float(wait_duration)
        self.intent_pub.publish(msg)

    def heartbeat_timer_callback(self):
        """Periodic node health heartbeat and full robot state broadcast."""
        hb = Heartbeat()
        hb.robot_id = self.robot_id
        hb.fleet_name = self.fleet_name
        hb.timestamp_tick = int(time.time() * 1000)
        hb.battery = float(self.battery_percent)
        hb.status = self.current_intent
        hb.current_task_id = self.current_task_id
        self.heartbeat_pub.publish(hb)

        # Publish full robot state for dashboard and peer coordination
        rs = FleetMindRobotState()
        rs.robot_id = self.robot_id
        rs.fleet_name = self.fleet_name
        rs.status = self.current_intent
        rs.position = Point(x=self.x, y=self.y, z=0.0)
        rs.velocity = float(math.hypot(self.vx, self.vy))
        rs.heading = float(self.yaw)
        rs.battery_percentage = float(self.battery_percent)
        rs.current_task_id = self.current_task_id
        rs.priority = int(self.task_priority)
        rs.waiting_for_id = self.current_target_peer
        rs.planned_path = []  # populated when path planner is active
        self.robot_state_pub.publish(rs)

    def prediction_timer_callback(self):
        """Edge AI decision loop: Risk evaluation and right-of-way arbitration."""
        current_time = time.time()

        # Skip evaluation if no recent state has been recorded
        if self.last_state_time > 0 and (current_time - self.last_state_time > 2.0):
            return

        # Peer tracking and stale/dead peer handling
        dead_peers = []
        for pid, peer in list(self.peers.items()):
            elapsed = current_time - peer['last_seen']
            if elapsed > self.lost_peer_sec:
                dead_peers.append(pid)
                self.recently_missing_peers[pid] = current_time
            elif elapsed > self.stale_peer_sec:
                # Stale peer: treat conservatively as static obstacle
                peer['vx'] = 0.0
                peer['vy'] = 0.0

        for pid in dead_peers:
            del self.peers[pid]

        self.recently_missing_peers = {
            pid: t for pid, t in self.recently_missing_peers.items()
            if current_time - t < 30.0
        }

        # Multi-peer risk evaluation
        critical_peer: Optional[str] = None
        warning_peer: Optional[str] = None
        min_critical_ttc = float('inf')
        min_crit_sep = float('inf')
        warn_ttc: Optional[float] = None
        warn_sep = float('inf')

        for peer_id, peer in self.peers.items():
            ttc, min_sep, risk, method = self.predictor.predict_risk(
                self.x, self.y, self.vx, self.vy,
                peer['x'], peer['y'], peer['vx'], peer['vy'],
                robot_radius=self.robot_radius,
                safety_margin=self.safety_margin,
                horizon=self.lookahead_horizon
            )

            if risk == "CRITICAL":
                effective_ttc = ttc if ttc is not None else 0.1
                if effective_ttc < min_critical_ttc:
                    min_critical_ttc = effective_ttc
                    min_crit_sep = min_sep
                    critical_peer = peer_id
            elif risk == "WARNING" and critical_peer is None:
                if not warning_peer:
                    warning_peer = peer_id
                    warn_ttc = ttc
                    warn_sep = min_sep

        # Right-of-Way Priority Negotiation
        decision_msg = SafetyDecision()
        decision_msg.robot_id = self.robot_id
        decision_msg.fleet_name = self.fleet_name

        if critical_peer:
            decision_msg.risk_level = "CRITICAL"
            decision_msg.time_to_collision = float(min_critical_ttc)
            decision_msg.min_separation = float(min_crit_sep)
            peer_data = self.peers[critical_peer]

            # Multi-Factor Arbitration:
            # 1. Higher task priority wins
            my_prio = self.task_priority
            peer_prio = peer_data.get('priority', 1)

            should_yield = False
            reason = ""

            if my_prio < peer_prio:
                should_yield = True
                reason = f"Lower task priority ({my_prio} vs {peer_prio}) to {critical_peer}"
            elif my_prio > peer_prio:
                should_yield = False
                reason = f"Higher task priority ({my_prio} vs {peer_prio}) over {critical_peer}"
            else:
                # 2. Wait time (the one who has been yielding longer gets priority to prevent starvation)
                my_wait = (current_time - self.yield_start_time) if self.current_intent == "YIELDING" else 0.0
                peer_wait = peer_data.get('wait_time_sec', 0.0)

                if peer_wait > my_wait + 2.0:
                    should_yield = True
                    reason = f"Peer {critical_peer} has waited longer ({peer_wait:.1f}s vs {my_wait:.1f}s)"
                elif my_wait > peer_wait + 2.0:
                    should_yield = False
                    reason = f"Waited longer than peer ({my_wait:.1f}s vs {peer_wait:.1f}s)"
                else:
                    # 3. Proximity to conflict / velocity dominance
                    my_speed = math.hypot(self.vx, self.vy)
                    peer_speed = math.hypot(peer_data['vx'], peer_data['vy'])

                    # 4. Deterministic tiebreak on lexicographical robot_id
                    if self.robot_id > critical_peer:
                        should_yield = True
                        reason = f"Yielding to {critical_peer} (deterministic tiebreak)"
                    else:
                        should_yield = False
                        reason = f"Priority over {critical_peer} (deterministic tiebreak)"

            if should_yield:
                if self.current_intent != "YIELDING":
                    self.yield_start_time = current_time
                self.yield_lock_target = critical_peer
                self.yield_lock_time = current_time
                self.current_intent = "YIELDING"
                self.current_target_peer = critical_peer
                decision_msg.decision = "YIELD"
                decision_msg.reason = reason
                decision_msg.target_peer = critical_peer
            else:
                self.yield_lock_target = None
                self.current_intent = "MOVING"
                self.current_target_peer = ""
                decision_msg.decision = "CONTINUE"
                decision_msg.reason = reason
                decision_msg.target_peer = critical_peer

        elif self.yield_lock_target and (current_time - self.yield_lock_time < self.yield_lockin_sec):
            # Hysteresis lock-in to prevent oscillations / chattering
            decision_msg.risk_level = "WARNING"
            decision_msg.decision = "YIELD"
            decision_msg.reason = f"Maintaining yield hysteresis lock-in for {self.yield_lock_target}"
            decision_msg.target_peer = self.yield_lock_target
            self.current_intent = "YIELDING"
            self.current_target_peer = self.yield_lock_target

        elif warning_peer:
            decision_msg.risk_level = "WARNING"
            decision_msg.decision = "MONITOR"
            decision_msg.reason = f"Monitoring potential trajectory risk with {warning_peer}"
            decision_msg.target_peer = warning_peer
            decision_msg.time_to_collision = float(warn_ttc) if warn_ttc else self.lookahead_horizon
            decision_msg.min_separation = float(warn_sep)
            self.yield_lock_target = None
            self.current_intent = "CAUTION"
            self.current_target_peer = warning_peer

        elif len(self.recently_missing_peers) > 0:
            # Conservative handling of dropped peers
            decision_msg.risk_level = "WARNING"
            decision_msg.decision = "MONITOR"
            decision_msg.reason = "Caution: Recently lost peer telemetry"
            decision_msg.target_peer = ""
            self.yield_lock_target = None
            self.current_intent = "CAUTION"
            self.current_target_peer = ""

        else:
            decision_msg.risk_level = "SAFE"
            decision_msg.decision = "CONTINUE"
            decision_msg.reason = "Trajectory clear"
            decision_msg.target_peer = ""
            self.yield_lock_target = None
            self.current_intent = "MOVING"
            self.current_target_peer = ""

        self.safety_pub.publish(decision_msg)
        self.global_safety_pub.publish(decision_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
