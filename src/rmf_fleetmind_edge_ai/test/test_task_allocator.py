import unittest
from rmf_fleetmind_edge_ai.distributed_task_allocator import estimate_edge_task_cost


class TestTaskAllocator(unittest.TestCase):

    def test_closer_robot_lower_cost(self):
        # Robot 1 at (0, 0), Robot 2 at (10, 10)
        # Task at (1, 1) -> (2, 2)
        cost1 = estimate_edge_task_cost(
            rx=0.0, ry=0.0,
            battery_percent=100.0,
            has_active_task=False,
            pickup_x=1.0, pickup_y=1.0,
            dropoff_x=2.0, dropoff_y=2.0
        )

        cost2 = estimate_edge_task_cost(
            rx=10.0, ry=10.0,
            battery_percent=100.0,
            has_active_task=False,
            pickup_x=1.0, pickup_y=1.0,
            dropoff_x=2.0, dropoff_y=2.0
        )

        self.assertLess(cost1, cost2)

    def test_low_battery_penalty(self):
        # Same position, but robot 2 has 10% battery (< 20%)
        cost_healthy = estimate_edge_task_cost(
            rx=0.0, ry=0.0, battery_percent=90.0, has_active_task=False,
            pickup_x=2.0, pickup_y=2.0, dropoff_x=4.0, dropoff_y=4.0
        )

        cost_low_batt = estimate_edge_task_cost(
            rx=0.0, ry=0.0, battery_percent=10.0, has_active_task=False,
            pickup_x=2.0, pickup_y=2.0, dropoff_x=4.0, dropoff_y=4.0
        )

        self.assertGreater(cost_low_batt, cost_healthy + 100.0)

    def test_busy_robot_penalty(self):
        # Idle robot should be preferred over busy robot at same distance
        cost_idle = estimate_edge_task_cost(
            rx=0.0, ry=0.0, battery_percent=80.0, has_active_task=False,
            pickup_x=3.0, pickup_y=3.0, dropoff_x=5.0, dropoff_y=5.0
        )

        cost_busy = estimate_edge_task_cost(
            rx=0.0, ry=0.0, battery_percent=80.0, has_active_task=True,
            pickup_x=3.0, pickup_y=3.0, dropoff_x=5.0, dropoff_y=5.0
        )

        self.assertGreater(cost_busy, cost_idle)


if __name__ == '__main__':
    unittest.main()
