#!/usr/bin/env python3
"""
FleetMind Fleet Coordinator - Multi-Fleet Introspection, Auto-Discovery & Telemetry.
Monitors all AMRs and Fleets via /fleet_states and /fleetmind/heartbeats,
tracking distributed topology, battery health, and active edge decisions.
"""
from __future__ import annotations

import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rmf_fleet_msgs.msg import FleetState
from fleetmind_msgs.msg import Heartbeat, SafetyDecision, PeerIntent, TaskOffer, ReplanRequest
from fleetmind_msgs.msg import RobotState as FleetMindRobotState
from std_msgs.msg import String
from geometry_msgs.msg import Point


class FleetCoordinator(Node):
    """
    Fleet-level coordinator providing auto-discovery and telemetry aggregation.
    """

    def __init__(self):
        super().__init__('fleet_coordinator')

        self.declare_parameter('report_interval_sec', 5.0)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('lost_peer_sec', 15.0)
        self.declare_parameter('deadlock_detection_enabled', True)

        self.report_interval_sec = float(self.get_parameter('report_interval_sec').value)
        self.low_battery_threshold = float(self.get_parameter('low_battery_threshold').value)
        self.lost_peer_sec = float(self.get_parameter('lost_peer_sec').value)
        self.deadlock_detection_enabled = bool(self.get_parameter('deadlock_detection_enabled').value)

        # Discovered fleets and robots: fleet_name -> set of robot_names
        self.discovered_fleets: Dict[str, Dict[str, Dict[str, Any]]] = {}
        self.recent_decisions: Dict[str, Dict[str, Any]] = {}
        self.last_known_tasks: Dict[str, str] = {}  # robot_id -> task_id
        self.reassigned_tasks: Dict[str, float] = {}  # task_id -> timestamp (prevent duplicates)

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        # Subscriptions
        self.fleet_states_sub = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_states_callback,
            qos_best_effort
        )

        self.heartbeat_sub = self.create_subscription(
            Heartbeat,
            '/fleetmind/heartbeats',
            self.heartbeat_callback,
            qos_reliable
        )

        self.safety_sub = self.create_subscription(
            SafetyDecision,
            '/fleetmind/safety_decision',
            self.safety_callback,
            qos_reliable
        )

        # Diagnostic publisher
        self.diag_pub = self.create_publisher(
            String,
            '/fleetmind/fleet_diagnostics',
            qos_reliable
        )

        # Task reassignment publisher
        self.task_offer_pub = self.create_publisher(
            TaskOffer,
            '/fleetmind/task_offers',
            qos_reliable
        )

        # Dynamic re-routing publisher
        self.replan_pub = self.create_publisher(
            ReplanRequest,
            '/fleetmind/replan_requests',
            qos_reliable
        )

        # Robot state subscriber for dashboard forwarding and deadlock detection
        self.robot_state_sub = self.create_subscription(
            FleetMindRobotState,
            '/fleetmind/robot_states',
            self.robot_state_callback,
            qos_reliable
        )

        # Status summary timer
        self.create_timer(self.report_interval_sec, self.report_timer_callback)

        self.get_logger().info("FleetMind Fleet Coordinator initialized and listening for fleets.")

    def fleet_states_callback(self, msg: FleetState):
        """Auto-discover fleets and AMRs from RMF /fleet_states."""
        fleet_name = msg.name
        if fleet_name not in self.discovered_fleets:
            self.discovered_fleets[fleet_name] = {}
            self.get_logger().info(f"[AUTO-DISCOVERY] Discovered new fleet: [{fleet_name}]")

        for robot in msg.robots:
            if robot.name not in self.discovered_fleets[fleet_name]:
                self.get_logger().info(
                    f"[AUTO-DISCOVERY] Discovered robot [{robot.name}] in fleet [{fleet_name}]"
                )

            self.discovered_fleets[fleet_name][robot.name] = {
                'battery': robot.battery_percent,
                'task_id': robot.task_id,
                'x': robot.location.x,
                'y': robot.location.y,
                'yaw': robot.location.yaw,
                'mode': robot.mode.mode,
                'last_seen': time.time()
            }

    def heartbeat_callback(self, msg: Heartbeat):
        """Track Edge-AI node heartbeats."""
        fleet_name = msg.fleet_name if msg.fleet_name else "default"
        if fleet_name not in self.discovered_fleets:
            self.discovered_fleets[fleet_name] = {}

        if msg.robot_id not in self.discovered_fleets[fleet_name]:
            self.discovered_fleets[fleet_name][msg.robot_id] = {}

        self.discovered_fleets[fleet_name][msg.robot_id].update({
            'battery': msg.battery,
            'status': msg.status,
            'task_id': msg.current_task_id,
            'edge_ai_active': True,
            'last_seen': time.time()
        })

    def safety_callback(self, msg: SafetyDecision):
        """Log and track active safety decisions."""
        self.recent_decisions[msg.robot_id] = {
            'decision': msg.decision,
            'reason': msg.reason,
            'target_peer': msg.target_peer,
            'risk_level': msg.risk_level,
            'ttc': msg.time_to_collision,
            'timestamp': time.time()
        }

    def report_timer_callback(self):
        """Generate periodic fleet telemetry report."""
        total_robots = sum(len(robots) for robots in self.discovered_fleets.values())
        if total_robots == 0:
            return

        lines = [f"=== FleetMind Fleet Telemetry Report ({total_robots} AMRs across {len(self.discovered_fleets)} fleets) ==="]
        for fleet, robots in self.discovered_fleets.items():
            lines.append(f"Fleet [{fleet}]:")
            for rname, rdata in robots.items():
                batt = rdata.get('battery', 100.0)
                status = rdata.get('status', 'ACTIVE')
                task = rdata.get('task_id', 'None')
                batt_warn = " [LOW BATTERY]" if batt < self.low_battery_threshold else ""

                dec_info = ""
                if rname in self.recent_decisions:
                    d = self.recent_decisions[rname]
                    if time.time() - d['timestamp'] < 5.0:
                        dec_info = f" | Decision: {d['decision']} ({d['reason']})"

                lines.append(f"  • {rname}: Battery={batt:.1f}%{batt_warn}, State={status}, Task={task}{dec_info}")

        summary = "\n".join(lines)
        self.get_logger().info(summary)

        msg = String()
        msg.data = summary
        self.diag_pub.publish(msg)

        # Detect unavailable robots and trigger task reassignment
        self.detect_unavailable_robots()

        # Advanced multi-agent deadlock detection
        if self.deadlock_detection_enabled:
            self.detect_deadlocks()

    def robot_state_callback(self, msg: FleetMindRobotState):
        """Track robot states for task reassignment and deadlock detection."""
        if msg.current_task_id:
            self.last_known_tasks[msg.robot_id] = msg.current_task_id
        
        # Store for deadlock detection
        fleet_name = msg.fleet_name
        if fleet_name not in self.discovered_fleets:
            self.discovered_fleets[fleet_name] = {}
        if msg.robot_id not in self.discovered_fleets[fleet_name]:
            self.discovered_fleets[fleet_name][msg.robot_id] = {}
        self.discovered_fleets[fleet_name][msg.robot_id]['waiting_for_id'] = msg.waiting_for_id

    def detect_deadlocks(self):
        """Detect circular wait dependencies (A -> B -> C -> A) and trigger replans."""
        wait_graph = {}
        for fleet, robots in self.discovered_fleets.items():
            for robot_id, data in robots.items():
                waiting_for = data.get('waiting_for_id', '')
                if waiting_for:
                    wait_graph[robot_id] = waiting_for

        visited = set()
        path = []

        def dfs(node):
            if node in path:
                # Cycle detected
                cycle_start = path.index(node)
                cycle = path[cycle_start:]
                self.get_logger().error(f"[DEADLOCK DETECTED] Circular wait cycle: {' -> '.join(cycle)} -> {node}")
                self.resolve_deadlock(cycle)
                return True
            if node in visited:
                return False
            
            visited.add(node)
            path.append(node)
            
            neighbor = wait_graph.get(node)
            if neighbor:
                if dfs(neighbor):
                    return True
                    
            path.pop()
            return False

        for node in wait_graph:
            if node not in visited:
                if dfs(node):
                    break # Resolve one cycle at a time

    def resolve_deadlock(self, cycle):
        """Break the deadlock by requesting a replan for one of the robots."""
        # Simple heuristic: pick the robot with the lexicographically smallest ID to replan
        target = min(cycle)
        self.get_logger().warn(f"[DEADLOCK RESOLUTION] Triggering dynamic re-route for {target} to break cycle.")
        
        req = ReplanRequest()
        req.robot_id = target
        req.fleet_name = "" # Broadcast, adapter filters by robot_id
        req.reason = "Cycle deadlock detected"
        req.expected_delay_sec = 10.0
        self.replan_pub.publish(req)

    def detect_unavailable_robots(self):
        """Detect robots that have gone missing and re-offer their tasks."""
        now = time.time()
        for fleet_name, robots in self.discovered_fleets.items():
            for rname, rdata in robots.items():
                last_seen = rdata.get('last_seen', now)
                elapsed = now - last_seen

                if elapsed > self.lost_peer_sec:
                    task_id = rdata.get('task_id', '') or self.last_known_tasks.get(rname, '')
                    if task_id and task_id not in self.reassigned_tasks:
                        self.get_logger().warn(
                            f"[FAULT RECOVERY] Robot [{rname}] in fleet [{fleet_name}] "
                            f"has been unreachable for {elapsed:.1f}s with active task [{task_id}]. "
                            f"Re-offering task for decentralized reassignment."
                        )
                        offer = TaskOffer()
                        offer.task_id = f"{task_id}_reassigned"
                        offer.task_type = 'delivery'
                        offer.pickup_location = Point(x=0.0, y=0.0, z=0.0)
                        offer.dropoff_location = Point(x=0.0, y=0.0, z=0.0)
                        offer.priority = 3  # elevated priority for reassignment
                        offer.deadline_sec = 300.0
                        self.task_offer_pub.publish(offer)
                        self.reassigned_tasks[task_id] = now

        # Clean old reassignment records (older than 5 minutes)
        self.reassigned_tasks = {
            tid: ts for tid, ts in self.reassigned_tasks.items()
            if now - ts < 300.0
        }


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
