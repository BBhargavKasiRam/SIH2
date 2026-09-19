#!/usr/bin/env python3
"""
FleetMind Dashboard Node — ROS 2 to WebSocket bridge for fleet monitoring.

Subscribes to FleetMind coordination topics and serves a lightweight web
dashboard via HTTP + Server-Sent Events (SSE). The dashboard is a monitoring-only
layer and is NOT a single point of failure — all robot coordination continues
independently via edge AI nodes even if this dashboard is offline.

Designed for edge-hardware deployability (Jetson Nano / Raspberry Pi).
"""
from __future__ import annotations

import json
import os
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from typing import Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rmf_fleet_msgs.msg import FleetState
from fleetmind_msgs.msg import (
    Heartbeat,
    SafetyDecision,
    RobotState as FleetMindRobotState,
)
from std_msgs.msg import String


# Global mutable state shared between ROS thread and HTTP thread
_dashboard_state: Dict[str, Any] = {
    'robots': {},
    'fleets': {},
    'safety_decisions': {},
    'diagnostics': '',
    'last_update': 0.0,
}
_state_lock = threading.Lock()
_sse_clients: List[Any] = []
_sse_lock = threading.Lock()


class DashboardHTTPHandler(SimpleHTTPRequestHandler):
    """Serves static files and provides SSE endpoint for real-time updates."""

    static_dir = ''

    def do_GET(self):
        if self.path == '/api/state':
            self._serve_json_state()
        elif self.path == '/api/sse':
            self._serve_sse()
        elif self.path == '/' or self.path == '/index.html':
            self._serve_static('index.html')
        elif self.path == '/style.css':
            self._serve_static('style.css')
        elif self.path == '/dashboard.js':
            self._serve_static('dashboard.js')
        else:
            self.send_error(404)

    def _serve_json_state(self):
        with _state_lock:
            data = json.dumps(_dashboard_state, default=str)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(data.encode())

    def _serve_sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()

        with _sse_lock:
            _sse_clients.append(self.wfile)

        try:
            while True:
                time.sleep(30)
                self.wfile.write(b': keepalive\n\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with _sse_lock:
                if self.wfile in _sse_clients:
                    _sse_clients.remove(self.wfile)

    def _serve_static(self, filename):
        filepath = os.path.join(self.static_dir, filename)
        if os.path.exists(filepath):
            content_types = {
                '.html': 'text/html',
                '.css': 'text/css',
                '.js': 'application/javascript',
            }
            ext = os.path.splitext(filename)[1]
            self.send_response(200)
            self.send_header('Content-Type', content_types.get(ext, 'text/plain'))
            self.end_headers()
            with open(filepath, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, f'File not found: {filename}')

    def log_message(self, format, *args):
        pass  # Suppress default HTTP logging noise


def broadcast_sse_event(event_type: str, data: dict):
    """Send an SSE event to all connected clients."""
    payload = f"event: {event_type}\ndata: {json.dumps(data, default=str)}\n\n"
    encoded = payload.encode()
    with _sse_lock:
        dead = []
        for client in _sse_clients:
            try:
                client.write(encoded)
                client.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                dead.append(client)
        for d in dead:
            _sse_clients.remove(d)


class DashboardNode(Node):
    """
    ROS 2 node that subscribes to FleetMind topics and feeds a web dashboard.
    """

    def __init__(self):
        super().__init__('fleetmind_dashboard')

        self.declare_parameter('http_port', 7860)
        self.declare_parameter('update_rate_hz', 2.0)

        self.http_port = int(self.get_parameter('http_port').value)
        update_rate = float(self.get_parameter('update_rate_hz').value)

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
        self.create_subscription(
            FleetMindRobotState,
            '/fleetmind/robot_states',
            self.robot_state_callback,
            qos_reliable
        )

        self.create_subscription(
            Heartbeat,
            '/fleetmind/heartbeats',
            self.heartbeat_callback,
            qos_reliable
        )

        self.create_subscription(
            SafetyDecision,
            '/fleetmind/safety_decision',
            self.safety_callback,
            qos_reliable
        )

        self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_state_callback,
            qos_best_effort
        )

        self.create_subscription(
            String,
            '/fleetmind/fleet_diagnostics',
            self.diagnostics_callback,
            qos_reliable
        )

        # Periodic SSE broadcast timer
        self.create_timer(1.0 / max(update_rate, 0.5), self.broadcast_timer_callback)

        # Start HTTP server in background thread
        self._start_http_server()

        self.get_logger().info(
            f"FleetMind Dashboard running at http://localhost:{self.http_port}"
        )

    def _start_http_server(self):
        # Determine static file directory
        try:
            from ament_index_python.packages import get_package_share_directory
            static_dir = os.path.join(
                get_package_share_directory('fleetmind_dashboard'), 'static'
            )
        except Exception:
            static_dir = os.path.join(os.path.dirname(__file__), '..', 'static')

        DashboardHTTPHandler.static_dir = static_dir

        server = HTTPServer(('0.0.0.0', self.http_port), DashboardHTTPHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

    def robot_state_callback(self, msg: FleetMindRobotState):
        robot_data = {
            'robot_id': msg.robot_id,
            'fleet_name': msg.fleet_name,
            'status': msg.status,
            'x': msg.position.x,
            'y': msg.position.y,
            'velocity': msg.velocity,
            'heading': msg.heading,
            'battery_percentage': msg.battery_percentage,
            'current_task_id': msg.current_task_id,
            'priority': msg.priority,
            'waiting_for_id': msg.waiting_for_id,
            'last_update': time.time(),
        }
        with _state_lock:
            _dashboard_state['robots'][msg.robot_id] = robot_data
            _dashboard_state['last_update'] = time.time()

    def heartbeat_callback(self, msg: Heartbeat):
        with _state_lock:
            if msg.robot_id not in _dashboard_state['robots']:
                _dashboard_state['robots'][msg.robot_id] = {}
            _dashboard_state['robots'][msg.robot_id].update({
                'robot_id': msg.robot_id,
                'fleet_name': msg.fleet_name,
                'battery_percentage': msg.battery,
                'status': msg.status,
                'current_task_id': msg.current_task_id,
                'last_update': time.time(),
            })
            _dashboard_state['last_update'] = time.time()

    def safety_callback(self, msg: SafetyDecision):
        decision_data = {
            'robot_id': msg.robot_id,
            'fleet_name': msg.fleet_name,
            'decision': msg.decision,
            'reason': msg.reason,
            'target_peer': msg.target_peer,
            'risk_level': msg.risk_level,
            'time_to_collision': msg.time_to_collision,
            'min_separation': msg.min_separation,
            'timestamp': time.time(),
        }
        with _state_lock:
            _dashboard_state['safety_decisions'][msg.robot_id] = decision_data
            _dashboard_state['last_update'] = time.time()

    def fleet_state_callback(self, msg: FleetState):
        fleet_data = {}
        for robot in msg.robots:
            fleet_data[robot.name] = {
                'x': robot.location.x,
                'y': robot.location.y,
                'yaw': robot.location.yaw,
                'battery': robot.battery_percent,
                'task_id': robot.task_id,
                'mode': robot.mode.mode,
            }
        with _state_lock:
            _dashboard_state['fleets'][msg.name] = fleet_data
            _dashboard_state['last_update'] = time.time()

    def diagnostics_callback(self, msg: String):
        with _state_lock:
            _dashboard_state['diagnostics'] = msg.data
            _dashboard_state['last_update'] = time.time()

    def broadcast_timer_callback(self):
        """Broadcast current state to all SSE clients."""
        with _state_lock:
            snapshot = json.loads(json.dumps(_dashboard_state, default=str))
        broadcast_sse_event('state_update', snapshot)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
