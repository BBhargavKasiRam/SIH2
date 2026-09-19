#!/usr/bin/env python3

import argparse
import glob
import math
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rmf_door_msgs.msg import DoorMode
from rmf_door_msgs.msg import DoorRequest
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import Location
from rmf_fleet_msgs.msg import PathRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import RobotState
from rmf_lift_msgs.msg import LiftRequest
from rmf_lift_msgs.msg import LiftState


class MockRobot:

    def __init__(self, name, model, x, y, yaw, level_name):
        self.name = name
        self.model = model
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.level_name = level_name

        self.task_id = "0"
        self.mode = RobotMode.MODE_IDLE
        self.battery = 100.0
        self.seq = 0
        self.path_points = []
        self.speed = 1.0  # m/s


class MockRobotSim(Node):

    def __init__(self, map_name='office'):
        super().__init__('mock_robot_sim')
        self.get_logger().info(f'Starting Mock Robot Simulator for map: {map_name}')

        self.robots = {}
        self.doors = {}  # door_name -> mode_int
        self.lifts = {}  # lift_name -> dict

        self.load_robots_from_map(map_name)

        # Robot state publisher & path subscriber
        self.robot_state_pub = self.create_publisher(
            RobotState, 'robot_state', 100
        )
        self.create_subscription(
            PathRequest,
            'robot_path_requests',
            self.path_request_cb,
            qos_profile_system_default,
        )

        # Door request & state
        self.door_state_pub = self.create_publisher(
            DoorState, 'door_states', 10
        )
        self.create_subscription(
            DoorRequest,
            'door_requests',
            self.door_request_cb,
            10,
        )

        # Lift request & state
        self.lift_state_pub = self.create_publisher(
            LiftState, 'lift_states', 10
        )
        self.create_subscription(
            LiftRequest,
            'lift_requests',
            self.lift_request_cb,
            10,
        )

        # 10 Hz simulation loop
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def load_robots_from_map(self, map_name):
        try:
            maps_pkg = get_package_share_directory('rmf_demos_maps')
            nav_files = sorted(glob.glob(f'{maps_pkg}/maps/{map_name}/nav_graphs/*.yaml'))
            for nav_file in nav_files:
                with open(nav_file, 'r') as f:
                    data = yaml.safe_load(f)

                for level, level_data in data.get('levels', {}).items():
                    for v in level_data.get('vertices', []):
                        params = v[2]
                        if 'spawn_robot_name' in params:
                            name = params['spawn_robot_name']
                            model = params.get('spawn_robot_type', 'tinyRobot')
                            x, y = v[0], v[1]
                            self.robots[name] = MockRobot(
                                name, model, x, y, 0.0, level
                            )
                            self.get_logger().info(
                                f'Loaded robot {name} at ({x:.2f}, {y:.2f}) on level {level}'
                            )
        except Exception as e:
            self.get_logger().warn(f'Could not load from nav_graph: {e}')

        # Fallback for office if not loaded
        if not self.robots and map_name == 'office':
            self.robots['tinyRobot1'] = MockRobot(
                'tinyRobot1', 'tinyRobot', 10.433, -5.575, 0.0, 'L1'
            )
            self.robots['tinyRobot2'] = MockRobot(
                'tinyRobot2', 'tinyRobot', 20.424, -5.312, 0.0, 'L1'
            )
            self.robots['tinyRobot3'] = MockRobot(
                'tinyRobot3', 'tinyRobot', 15.0, -8.0, 0.0, 'L1'
            )
            self.get_logger().info('Loaded default office tinyRobots (3 AMRs)')

    def path_request_cb(self, msg: PathRequest):
        if msg.robot_name in self.robots:
            robot = self.robots[msg.robot_name]
            robot.task_id = str(msg.task_id)
            robot.path_points = list(msg.path)
            if len(robot.path_points) > 0:
                robot.mode = RobotMode.MODE_MOVING
                self.get_logger().info(
                    f'[{robot.name}] Received path request {robot.task_id} with {len(robot.path_points)} waypoints'
                )

    def door_request_cb(self, msg: DoorRequest):
        self.get_logger().info(
            f'Door request received for {msg.door_name}: mode {msg.requested_mode.value}'
        )
        # Automatically fulfill door open/close requests
        self.doors[msg.door_name] = msg.requested_mode.value
        self.publish_door_state(msg.door_name, msg.requested_mode.value)

    def publish_door_state(self, door_name, mode_val):
        state = DoorState()
        state.door_time = self.get_clock().now().to_msg()
        state.door_name = door_name
        state.current_mode.value = mode_val
        self.door_state_pub.publish(state)

    def lift_request_cb(self, msg: LiftRequest):
        self.get_logger().info(
            f'Lift request received for {msg.lift_name}: to floor {msg.destination_floor}'
        )
        self.lifts[msg.lift_name] = {
            'current_floor': msg.destination_floor,
            'destination_floor': msg.destination_floor,
            'door_state': LiftState.DOOR_OPEN,
            'session_id': msg.session_id,
        }
        self.publish_lift_state(msg.lift_name)

    def publish_lift_state(self, lift_name):
        data = self.lifts[lift_name]
        state = LiftState()
        state.lift_time = self.get_clock().now().to_msg()
        state.lift_name = lift_name
        state.current_floor = data['current_floor']
        state.destination_floor = data['destination_floor']
        state.door_state = data['door_state']
        state.motion_state = LiftState.MOTION_STOPPED
        state.current_mode = LiftState.MODE_AGV
        state.session_id = data['session_id']
        self.lift_state_pub.publish(state)

    def timer_cb(self):
        now_msg = self.get_clock().now().to_msg()

        # 1. Update robots
        for robot in self.robots.values():
            if robot.path_points:
                target = robot.path_points[0]
                dx = target.x - robot.x
                dy = target.y - robot.y
                dist = math.hypot(dx, dy)

                step = robot.speed * self.dt
                if dist <= step:
                    robot.x = float(target.x)
                    robot.y = float(target.y)
                    robot.yaw = float(target.yaw)
                    robot.level_name = target.level_name
                    robot.path_points.pop(0)

                    if not robot.path_points:
                        robot.mode = RobotMode.MODE_IDLE
                        self.get_logger().info(
                            f'[{robot.name}] Reached destination for cmd {robot.task_id}'
                        )
                else:
                    robot.x += (dx / dist) * step
                    robot.y += (dy / dist) * step
                    robot.yaw = math.atan2(dy, dx)
                    robot.level_name = target.level_name

            # Publish RobotState
            state = RobotState()
            state.name = robot.name
            state.model = robot.model
            state.task_id = str(robot.task_id)
            state.seq = robot.seq
            robot.seq += 1

            state.mode.mode = robot.mode
            state.battery_percent = float(robot.battery)

            state.location.x = float(robot.x)
            state.location.y = float(robot.y)
            state.location.yaw = float(robot.yaw)
            state.location.level_name = robot.level_name
            state.location.t = now_msg

            # Remaining path
            state.path = list(robot.path_points)

            self.robot_state_pub.publish(state)

        # 2. Keep broadcasting door states so supervisor knows doors are open
        for door_name, mode_val in self.doors.items():
            self.publish_door_state(door_name, mode_val)

        # 3. Keep broadcasting lift states
        for lift_name in self.lifts:
            self.publish_lift_state(lift_name)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--map', default='office', help='Map name')
    parsed_args, _ = parser.parse_known_args()

    node = MockRobotSim(parsed_args.map)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
