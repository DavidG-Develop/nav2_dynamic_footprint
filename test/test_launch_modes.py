"""Exercise every launch mode, including loading into an existing container."""
from pathlib import Path
import tempfile
import time
import unittest

from geometry_msgs.msg import Polygon, PolygonStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy


MODES = [
    ('plain_standalone', False, False, False),
    ('managed_standalone', True, False, False),
    ('plain_component', False, True, False),
    ('managed_component', True, True, False),
    ('plain_existing', False, True, True),
    ('managed_existing', True, True, True),
]


@pytest.mark.launch_test
def generate_test_description():
    temporary = tempfile.TemporaryDirectory(prefix='footprint-launch-test-')
    urdf = Path(temporary.name) / 'robot.urdf'
    urdf.write_text("<robot name='test'><link name='base_link'><collision><geometry>"
                    "<box size='2 1 1'/></geometry></collision></link></robot>")
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'dynamic_footprint.launch.py'
    actions = [ComposableNodeContainer(
        package='rclcpp_components', executable='component_container',
        name='shared_container', namespace='')]
    for name, lifecycle, composed, existing in MODES:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)), launch_arguments={
                'namespace': name,
                'use_lifecycle': str(lifecycle).lower(),
                'use_composition': str(composed).lower(),
                'use_stamped_polygon': str(lifecycle or composed).lower(),
                'container_name': '/shared_container' if existing else '',
                'urdf': str(urdf), 'footprint_topic': 'footprint',
            }.items()))
    return LaunchDescription(actions + [launch_testing.actions.ReadyToTest()]), {
        'temporary': temporary,
    }


class TestModes(unittest.TestCase):

    def test_modes(self):
        rclpy.init()
        node = rclpy.create_node('launch_modes_observer')
        counts = {}
        messages = {}

        def receive(name, msg):
            counts[name] = counts.get(name, 0) + 1
            messages[name] = msg

        subscriptions = [node.create_subscription(
            PolygonStamped if lifecycle or composed else Polygon, f'/{name}/footprint',
            lambda msg, name=name: receive(name, msg), 10)
            for name, lifecycle, composed, _ in MODES]
        self.assertEqual(len(subscriptions), len(MODES))

        def wait(predicate):
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return True
            return False

        def transition(client, transition_id):
            request = ChangeState.Request()
            request.transition.id = transition_id
            future = client.call_async(request)
            self.assertTrue(wait(future.done))
            self.assertTrue(future.result().success)

        try:
            self.assertTrue(wait(lambda: all(
                name in messages for name, lifecycle, *_ in MODES if not lifecycle)))
            clients = {}
            for name, lifecycle, *_ in MODES:
                if lifecycle:
                    self.assertNotIn(name, messages)
                    client = node.create_client(
                        ChangeState, f'/{name}/dynamic_footprint/change_state')
                    self.assertTrue(wait(client.service_is_ready))
                    transition(client, 1)  # Configure, still no publication.
                    clients[name] = client
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            for name in clients:
                self.assertNotIn(name, messages)
                transition(clients[name], 3)  # Activate.
            self.assertTrue(wait(lambda: len(messages) == len(MODES)))
            for msg in messages.values():
                if isinstance(msg, PolygonStamped):
                    self.assertEqual(msg.header.frame_id, 'base_link')
                    self.assertGreater(msg.header.stamp.sec, 0)
                    msg = msg.polygon
                self.assertEqual(len(msg.points), 4)
                self.assertAlmostEqual(max(p.x for p in msg.points), 1.0)
                self.assertAlmostEqual(max(p.y for p in msg.points), 0.5)
            for client in clients.values():
                transition(client, 4)  # Deactivate.
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            before = {name: counts[name] for name in clients}
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(before, {name: counts[name] for name in clients})
            for client in clients.values():
                transition(client, 2)  # Cleanup.
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):

    def test_exit(self, proc_info, temporary):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        finally:
            temporary.cleanup()
