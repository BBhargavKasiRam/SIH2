#!/usr/bin/env python3
"""
FleetMind Distributed Task Allocator & Dynamic Reassignment Engine.
Computes multi-attribute cost estimates for task bidding at the edge
and facilitates decentralized dynamic task reassignment upon robot fault.
"""
from __future__ import annotations

import math
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from fleetmind_msgs.msg import TaskOffer, TaskBid, ObstacleAlert, Heartbeat
from nav_msgs.msg import Odometry
from rmf_fleet_msgs.msg import FleetState


def estimate_edge_task_cost(
    rx: float, ry: float,
    battery_percent: float,
    has_active_task: bool,
    pickup_x: float, pickup_y: float,
    dropoff_x: float, dropoff_y: float,
    task_priority: int = 1,
    nominal_speed: float = 0.8
) -> float:
    """
    Computes decentralized task execution cost.
    Lower cost = better candidate.
    """
    dist_to_pickup = math.hypot(pickup_x - rx, pickup_y - ry)
    dist_pickup_to_dropoff = math.hypot(dropoff_x - pickup_x, dropoff_y - pickup_y)
    total_distance = dist_to_pickup + dist_pickup_to_dropoff

    # Distance cost proportional to travel time
    travel_cost = total_distance / max(nominal_speed, 0.1)

    # Battery penalty: severe penalty if under 20%
    battery_penalty = 150.0 if battery_percent < 20.0 else (100.0 - battery_percent) * 0.2

    # Workload penalty if currently engaged
    workload_penalty = 60.0 if has_active_task else 0.0

    # Urgency bonus (higher priority reduces effective cost to prioritize fast dispatch)
    urgency_bonus = -15.0 * task_priority

    return float(travel_cost + battery_penalty + workload_penalty + urgency_bonus)


class DistributedTaskAllocator(Node):
    """
    Autonomous task bidding and reassignment node running onboard an AMR.
    """

    def __init__(self):
        super().__init__('distributed_task_allocator')

        self.declare_parameter('robot_id', 'tinyRobot1')
        self.declare_parameter('fleet_name', 'tinyRobot')
        self.declare_parameter('nominal_speed', 0.8)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.fleet_name = str(self.get_parameter('fleet_name').value)
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)

        self.x = 0.0
        self.y = 0.0
        self.battery_percent = 100.0
        self.current_task_id = ""

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
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

        self.fleet_states_sub = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_states_callback,
            qos_best_effort
        )

        self.task_offer_sub = self.create_subscription(
            TaskOffer,
            '/fleetmind/task_offers',
            self.task_offer_callback,
            qos_reliable
        )

        self.obstacle_sub = self.create_subscription(
            ObstacleAlert,
            '/fleetmind/obstacle_alerts',
            self.obstacle_alert_callback,
            qos_reliable
        )

        # Publishers
        self.task_bid_pub = self.create_publisher(
            TaskBid,
            '/fleetmind/task_bids',
            qos_reliable
        )

        self.get_logger().info(f"Distributed Task Allocator running for [{self.fleet_name}/{self.robot_id}]")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def fleet_states_callback(self, msg: FleetState):
        for robot in msg.robots:
            if robot.name == self.robot_id:
                self.x = robot.location.x
                self.y = robot.location.y
                self.battery_percent = robot.battery_percent
                self.current_task_id = robot.task_id

    def task_offer_callback(self, msg: TaskOffer):
        """Evaluate task offer and submit an edge cost bid."""
        has_task = bool(self.current_task_id)

        cost = estimate_edge_task_cost(
            self.x, self.y,
            self.battery_percent,
            has_task,
            msg.pickup_location.x, msg.pickup_location.y,
            msg.dropoff_location.x, msg.dropoff_location.y,
            task_priority=msg.priority,
            nominal_speed=self.nominal_speed
        )

        dist_total = (
            math.hypot(msg.pickup_location.x - self.x, msg.pickup_location.y - self.y) +
            math.hypot(msg.dropoff_location.x - msg.pickup_location.x, msg.dropoff_location.y - msg.pickup_location.y)
        )
        eta = dist_total / max(self.nominal_speed, 0.1)

        bid = TaskBid()
        bid.task_id = msg.task_id
        bid.robot_id = self.robot_id
        bid.fleet_name = self.fleet_name
        bid.bid_cost = float(cost)
        bid.eta_seconds = float(eta)
        bid.battery_percent = float(self.battery_percent)
        bid.can_execute = (self.battery_percent >= 15.0)

        self.get_logger().info(
            f"[TASK BID] Submitting bid for {msg.task_id}: Cost={cost:.2f}, ETA={eta:.1f}s, "
            f"Battery={self.battery_percent:.1f}%"
        )
        self.task_bid_pub.publish(bid)

    def obstacle_alert_callback(self, msg: ObstacleAlert):
        """Handle severe obstacle alert affecting current route."""
        if not self.current_task_id:
            return

        # Check proximity to reported blocked cells
        for pt in msg.blocked_cells:
            dist = math.hypot(pt.x - self.x, pt.y - self.y)
            if dist < msg.radius + 1.0:
                self.get_logger().warn(
                    f"[DYNAMIC REASSIGNMENT] Active task {self.current_task_id} blocked by "
                    f"obstacle at ({pt.x:.1f}, {pt.y:.1f})! Alerting peer fleet for reassignment."
                )
                break


def main(args=None):
    rclpy.init(args=args)
    node = DistributedTaskAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
