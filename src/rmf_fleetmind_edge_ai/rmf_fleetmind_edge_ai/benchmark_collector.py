#!/usr/bin/env python3
"""
FleetMind Benchmark Collector - Collects performance metrics for SIH-26123.
Tracks total task time, idle time, wait/yield time, conflicts, deadlocks.
"""
from __future__ import annotations

import json
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from fleetmind_msgs.msg import SafetyDecision
from fleetmind_msgs.msg import RobotState as FleetMindRobotState


class BenchmarkCollector(Node):
    """
    Collects performance metrics for comparison against Stop-and-Wait coordination.
    """

    def __init__(self):
        super().__init__('benchmark_collector')

        self.declare_parameter('enabled', False)
        self.declare_parameter('output_file', '/tmp/fleetmind_benchmark.json')
        self.declare_parameter('report_interval_sec', 10.0)

        if not self.get_parameter('enabled').value:
            self.get_logger().info("Benchmark Collector is disabled.")
            return

        self.output_file = str(self.get_parameter('output_file').value)
        self.report_interval = float(self.get_parameter('report_interval_sec').value)

        self.metrics: Dict[str, Any] = {
            'start_time': time.time(),
            'total_conflicts_warning': 0,
            'total_conflicts_critical': 0,
            'total_deadlocks_detected': 0,
            'total_replans_triggered': 0,
            'robots': {}
        }

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.create_subscription(
            FleetMindRobotState,
            '/fleetmind/robot_states',
            self.robot_state_callback,
            qos_reliable
        )

        self.create_subscription(
            SafetyDecision,
            '/fleetmind/safety_decision',
            self.safety_callback,
            qos_reliable
        )

        # Deadlock / Replan counting via diagnostic/replan topics could be added here
        
        self.create_timer(self.report_interval, self.report_timer_callback)
        self.get_logger().info(f"Benchmark Collector started, saving to {self.output_file}")

    def robot_state_callback(self, msg: FleetMindRobotState):
        if msg.robot_id not in self.metrics['robots']:
            self.metrics['robots'][msg.robot_id] = {
                'tasks_completed': 0,
                'total_yield_time_sec': 0.0,
                'total_moving_time_sec': 0.0,
                'total_idle_time_sec': 0.0,
                'last_update': time.time(),
                'last_status': msg.status,
                'last_task': msg.current_task_id
            }

        r_metrics = self.metrics['robots'][msg.robot_id]
        now = time.time()
        dt = now - r_metrics['last_update']

        if r_metrics['last_status'] == 'YIELDING':
            r_metrics['total_yield_time_sec'] += dt
        elif r_metrics['last_status'] == 'MOVING':
            r_metrics['total_moving_time_sec'] += dt
        else:
            r_metrics['total_idle_time_sec'] += dt

        if msg.current_task_id != r_metrics['last_task'] and r_metrics['last_task']:
            r_metrics['tasks_completed'] += 1

        r_metrics['last_status'] = msg.status
        r_metrics['last_task'] = msg.current_task_id
        r_metrics['last_update'] = now

    def safety_callback(self, msg: SafetyDecision):
        if msg.risk_level == 'WARNING':
            self.metrics['total_conflicts_warning'] += 1
        elif msg.risk_level == 'CRITICAL':
            self.metrics['total_conflicts_critical'] += 1

    def report_timer_callback(self):
        self.metrics['uptime_sec'] = time.time() - self.metrics['start_time']
        try:
            with open(self.output_file, 'w') as f:
                json.dump(self.metrics, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to write benchmark: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkCollector()
    if node.get_parameter('enabled').value:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
