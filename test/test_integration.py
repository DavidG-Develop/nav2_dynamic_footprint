"""Exercise real component loading, standalone startup, URDF/TF and Nav2 input."""
import math
import time
import unittest

from geometry_msgs.msg import Polygon, PolygonStamped
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.srv import ChangeState
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


URDF = """<robot name="test">
<link name="base_link"><collision><geometry>
<box size="1 1 0.2"/></geometry></collision></link>
<link name="arm"><collision><origin xyz="1 0 0" rpy="0 0 0"/>
<geometry><box size="2 0.2 0.2"/></geometry></collision></link>
<joint name="hinge" type="continuous"><parent link="base_link"/>
<child link="arm"/><axis xyz="0 0 1"/></joint></robot>"""


@pytest.mark.launch_test
def generate_test_description():
    parameters = {'robot_description': URDF, 'max_transform_age': 0.2}
    standalone = Node(
        package='nav2_dynamic_footprint', executable='dynamic_footprint_node',
        name='standalone', parameters=[{
            'max_transform_age': 0.2, 'footprint_topic': '/costmap/footprint'}])
    container = ComposableNodeContainer(
        name='test_container', namespace='', package='rclcpp_components',
        executable='component_container', composable_node_descriptions=[ComposableNode(
            package='nav2_dynamic_footprint',
            plugin='nav2_dynamic_footprint::DynamicFootprint', name='composed',
            parameters=[parameters, {'footprint_topic': '/composed_footprint'}]), ComposableNode(
            package='nav2_dynamic_footprint',
            plugin='nav2_dynamic_footprint::DynamicFootprintLifecycle', name='managed_component',
            parameters=[{'max_transform_age': 0.2,
                         'footprint_topic': '/managed_component_footprint'}])])
    managed_standalone = Node(
        package='nav2_dynamic_footprint', executable='dynamic_footprint_lifecycle_node',
        name='managed_standalone',
        parameters=[parameters, {'footprint_topic': '/managed_standalone_footprint'}])
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='test_manager',
        parameters=[{'autostart': True,
                     'node_names': ['managed_component', 'managed_standalone']}])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': URDF}])
    costmap = Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d', name='costmap',
        remappings=[('footprint', '/costmap/footprint'),
                    ('published_footprint', '/costmap/published_footprint')],
        parameters=[{
            'global_frame': 'base_link', 'robot_base_frame': 'base_link',
            'rolling_window': True, 'width': 10, 'height': 10,
            'resolution': 0.1, 'update_frequency': 20.0, 'publish_frequency': 20.0,
            'footprint_padding': 0.0, 'plugins': ['inflation_layer'],
            'inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
        }])
    return LaunchDescription([
        standalone, container, rsp, costmap, managed_standalone, manager,
        launch_testing.actions.ReadyToTest(),
    ]), {'standalone': standalone, 'container': container, 'rsp': rsp, 'costmap': costmap}


class TestFootprint(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_dynamic_geometry_and_nav2(self, proc_output, standalone):
        node = rclpy.create_node('footprint_test')
        messages = {}
        counts = {}

        def receive(key, msg):
            messages[key] = msg
            counts[key] = counts.get(key, 0) + 1

        subscriptions = []
        for topic, msg_type in [('/costmap/footprint', Polygon),
                                ('/composed_footprint', Polygon),
                                ('/managed_component_footprint', Polygon),
                                ('/managed_standalone_footprint', Polygon),
                                ('/costmap/published_footprint', PolygonStamped)]:
            subscriptions.append(node.create_subscription(
                msg_type, topic, lambda msg, key=topic: receive(key, msg), 10))
        joints = node.create_publisher(JointState, '/joint_states', 10)
        client = node.create_client(ChangeState, '/costmap/change_state')
        manager_client = node.create_client(Trigger, '/test_manager/is_active')
        angle = 0.0

        def tick():
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = ['hinge']
            msg.position = [angle]
            joints.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

        def wait(predicate, timeout=20.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                tick()
                if predicate():
                    return True
            return False

        def bounds(msg):
            polygon = msg.polygon if isinstance(msg, PolygonStamped) else msg
            return (min(p.x for p in polygon.points), max(p.x for p in polygon.points),
                    min(p.y for p in polygon.points), max(p.y for p in polygon.points))

        try:
            # Without joint TF neither publisher may emit a partial base-only hull.
            deadline = time.monotonic() + 2.4
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertNotIn('/costmap/footprint', messages)
            self.assertNotIn('/composed_footprint', messages)
            proc_output.assertWaitFor('Waiting for first valid footprint:', process=standalone)
            proc_output.assertWaitFor('Footprint still withheld after', process=standalone)
            self.assertTrue(wait(lambda: client.service_is_ready()))
            for transition in [1, 3]:  # configure, activate
                request = ChangeState.Request()
                request.transition.id = transition
                future = client.call_async(request)
                self.assertTrue(wait(future.done))
                self.assertTrue(future.result().success)
            self.assertTrue(wait(lambda: manager_client.service_is_ready()))
            active = manager_client.call_async(Trigger.Request())
            self.assertTrue(wait(active.done))
            self.assertTrue(active.result().success, 'Nav2 manager failed to form lifecycle bonds')
            topics = ['/costmap/footprint', '/composed_footprint',
                      '/managed_component_footprint', '/managed_standalone_footprint',
                      '/costmap/published_footprint']
            self.assertTrue(wait(lambda: all(
                t in messages and abs(bounds(messages[t])[1] - 2.0) < 0.001
                for t in topics)))
            proc_output.assertWaitFor('Footprint publishing started', process=standalone)
            for topic in topics:
                for actual, expected in zip(bounds(messages[topic]), [-0.5, 2.0, -0.5, 0.5]):
                    self.assertAlmostEqual(actual, expected, places=3)
            angle = math.pi / 2
            self.assertTrue(wait(lambda: all(
                abs(bounds(messages[t])[3] - 2.0) < 0.001 for t in topics)))
            for topic in topics:
                for actual, expected in zip(bounds(messages[topic]), [-0.5, 0.5, -0.5, 2.0]):
                    self.assertAlmostEqual(actual, expected, places=3)
            # Stop joint updates, drain in-flight samples, then verify publication stops.
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            before = [counts[t] for t in topics[:-1]]
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertEqual(before, [counts[t] for t in topics[:-1]])
            proc_output.assertWaitFor('Footprint publishing interrupted:', process=standalone)
            angle = 0.0
            self.assertTrue(wait(lambda: all(
                abs(bounds(messages[t])[1] - 2.0) < 0.001 for t in topics)))
            proc_output.assertWaitFor('Footprint publishing resumed', process=standalone)
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):

    def test_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
