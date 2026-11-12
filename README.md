# nav2_dynamic_footprint

A C++17 ROS 2 component and standalone node that publish a dynamic Nav2 footprint
from URDF collision geometry and TF supplied by `robot_state_publisher`.

## Distribution support

| Distribution | Status |
|--------------|--------|
| Jazzy | [![Jazzy](https://github.com/DavidG-Develop/nav2_dynamic_footprint/actions/workflows/ros2-status-jazzy.yaml/badge.svg?branch=main)](https://github.com/DavidG-Develop/nav2_dynamic_footprint/actions/workflows/ros2-ci.yaml) |
| Lyrical | [![Lyrical](https://github.com/DavidG-Develop/nav2_dynamic_footprint/actions/workflows/ros2-status-lyrical.yaml/badge.svg?branch=main)](https://github.com/DavidG-Develop/nav2_dynamic_footprint/actions/workflows/ros2-ci.yaml) |
| Rolling | [![Rolling](https://github.com/DavidG-Develop/nav2_dynamic_footprint/actions/workflows/ros2-status-rolling.yaml/badge.svg?branch=main)](https://github.com/DavidG-Develop/nav2_dynamic_footprint/actions/workflows/ros2-ci.yaml) |

## Installation

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"
colcon build --packages-select nav2_dynamic_footprint --symlink-install
source install/setup.bash
```

## Usage

With `robot_state_publisher` running, the node subscribes to its `robot_description`
topic and TF:

```bash
ros2 launch nav2_dynamic_footprint dynamic_footprint.launch.py
```

Alternatively, add `urdf:=/absolute/path/to/robot.urdf` to load a file. An explicit
`robot_description` parameter takes precedence over the topic. Use
`robot_description_topic:=/robot/robot_description` for a namespaced publisher.

Component: `nav2_dynamic_footprint::DynamicFootprint` (`rclcpp::NodeOptions`).
Standalone: `dynamic_footprint_node`. Output: `geometry_msgs/msg/Polygon` in
`base_frame`; match Nav2's `robot_base_frame`.

Parameter | Default
----------|--------
robot_description | Empty (subscribe to topic)
robot_description_topic | robot_description
base_frame | base_link
footprint_topic | dynamic_footprint
publish_rate | 10.0 Hz
max_transform_age | 0.5 s

`publish_rate` and `max_transform_age` can be changed at runtime; invalid changes
are rejected. Frame, topic, and description-source parameters are read-only.
See [example configuration](config/footprint.yaml).

```bash
ros2 param set /dynamic_footprint publish_rate 20.0
ros2 param set /dynamic_footprint max_transform_age 0.3
```
The public `convex_hull(std::vector<Point>)` API returns a counterclockwise hull
and rejects non-finite or degenerate input.

All collision geometry contributes to a convex XY envelope. Boxes and meshes use
vertices; spheres and cylinders use conservative bounding prisms. Missing or stale
TF withholds the entire update; Nav2 retains its previous footprint.

## Nav2 integration

One publisher serves both costmaps on `/dynamic_footprint`. Add these remappings
to the Nav2 `controller_server` and `planner_server` launch definitions
(`Node` or `ComposableNode`):

```python
remappings=[
    ('/local_costmap/footprint', '/dynamic_footprint'),
    ('/global_costmap/footprint', '/dynamic_footprint'),
]
```

Set both costmaps' `robot_base_frame` to this node's `base_frame` (default
`base_link`). Their `global_frame` can differ (`odom` locally, `map` globally);
Nav2 transforms the shared footprint using TF. Adjust the remapping paths if
Nav2 runs under a namespace.

### Launch modes

One launch file supports all four modes:

Mode | use_lifecycle | use_composition
-----|---------------|----------------
Plain standalone (default) | false | false
Plain component | false | true
Lifecycle standalone | true | false
Lifecycle component | true | true

For a managed component:

```bash
ros2 launch nav2_dynamic_footprint dynamic_footprint.launch.py \
  use_lifecycle:=true use_composition:=true
```

Composition starts a dedicated container by default. Add
`container_name:=/nav2_container` to load into an existing container instead.
Use `namespace:=robot` to namespace the footprint node.

For lifecycle modes, add `dynamic_footprint` to your Nav2 lifecycle manager's
`node_names`, before the costmap hosts. The managed node supports Nav2 bonds and publishes only while
active; cleanup releases its resources. `use_lifecycle` is a launch argument,
not a node parameter. For composition, select
`nav2_dynamic_footprint::DynamicFootprintLifecycle`; its standalone executable is
`dynamic_footprint_lifecycle_node`. Both modes use the same implementation.
