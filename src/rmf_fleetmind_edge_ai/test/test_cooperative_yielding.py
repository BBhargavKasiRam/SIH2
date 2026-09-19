import unittest
import time
import rclpy
from rmf_fleetmind_edge_ai.edge_ai_node import EdgeAINode


class MockPublisher:
    def __init__(self):
        self.last_msg = None

    def publish(self, msg):
        self.last_msg = msg


class TestCooperativeYielding(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = EdgeAINode()
        self.node.safety_pub = MockPublisher()
        self.node.intent_pub = MockPublisher()

    def tearDown(self):
        self.node.destroy_node()

    def inject_state(self, robot_id, x, y, vx, vy, prio=1, wait_sec=0.0):
        self.node.robot_id = robot_id
        self.node.x = x
        self.node.y = y
        self.node.vx = vx
        self.node.vy = vy
        self.node.task_priority = prio
        self.node.last_state_time = time.time()
        if wait_sec > 0:
            self.node.current_intent = "YIELDING"
            self.node.yield_start_time = time.time() - wait_sec
        else:
            self.node.current_intent = "MOVING"

    def inject_peer(self, peer_id, x, y, vx, vy, prio=1, wait_sec=0.0, age=0.0):
        self.node.peers[peer_id] = {
            'fleet_name': 'tinyRobot',
            'x': x,
            'y': y,
            'vx': vx,
            'vy': vy,
            'intent': 'MOVING' if wait_sec == 0 else 'YIELDING',
            'target_peer': '',
            'priority': prio,
            'wait_time_sec': wait_sec,
            'last_seen': time.time() - age
        }

    def test_task_priority_arbitration(self):
        # AMR 1 (prio 2 - high) vs AMR 2 (prio 1 - normal)
        self.inject_state("tinyRobot2", 0.0, 0.0, 1.0, 0.0, prio=1)
        self.inject_peer("tinyRobot1", 2.0, 0.0, -1.0, 0.0, prio=2)

        self.node.prediction_timer_callback()
        dec = self.node.safety_pub.last_msg
        self.assertIsNotNone(dec)
        self.assertEqual(dec.decision, "YIELD")
        self.assertEqual(dec.target_peer, "tinyRobot1")

        # Reverse perspective: tinyRobot1 has higher priority, should CONTINUE
        self.inject_state("tinyRobot1", 2.0, 0.0, -1.0, 0.0, prio=2)
        self.inject_peer("tinyRobot2", 0.0, 0.0, 1.0, 0.0, prio=1)

        self.node.prediction_timer_callback()
        dec = self.node.safety_pub.last_msg
        self.assertEqual(dec.decision, "CONTINUE")

    def test_starvation_prevention_wait_time(self):
        # Equal priority, but tinyRobot2 has been yielding for 10s, tinyRobot1 just arrived (0s)
        self.inject_state("tinyRobot1", 2.0, 0.0, -1.0, 0.0, prio=1, wait_sec=0.0)
        self.inject_peer("tinyRobot2", 0.0, 0.0, 1.0, 0.0, prio=1, wait_sec=10.0)

        self.node.prediction_timer_callback()
        dec = self.node.safety_pub.last_msg
        self.assertEqual(dec.decision, "YIELD")
        self.assertEqual(dec.target_peer, "tinyRobot2")

    def test_lexicographical_tiebreak(self):
        # Equal priority, equal wait time -> lexicographical: tinyRobot2 > tinyRobot1 -> tinyRobot2 yields
        self.inject_state("tinyRobot2", 0.0, 0.0, 1.0, 0.0, prio=1, wait_sec=0.0)
        self.inject_peer("tinyRobot1", 2.0, 0.0, -1.0, 0.0, prio=1, wait_sec=0.0)

        self.node.prediction_timer_callback()
        dec = self.node.safety_pub.last_msg
        self.assertEqual(dec.decision, "YIELD")

    def test_anti_oscillation_hysteresis(self):
        # After yielding, clearing peer immediately shouldn't cause instant toggle
        self.node.yield_lock_target = "tinyRobot1"
        self.node.yield_lock_time = time.time()  # just locked
        self.node.peers.clear()  # peer moved away

        self.node.prediction_timer_callback()
        dec = self.node.safety_pub.last_msg
        self.assertEqual(dec.decision, "YIELD")
        self.assertIn("hysteresis", dec.reason.lower())


if __name__ == '__main__':
    unittest.main()
