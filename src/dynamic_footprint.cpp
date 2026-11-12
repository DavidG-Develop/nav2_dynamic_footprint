#include "nav2_dynamic_footprint/dynamic_footprint.hpp"
#include "nav2_dynamic_footprint/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "geometric_shapes/shape_operations.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/LinearMath/Transform.hpp"
#if __has_include("urdf/model.hpp")
#include "urdf/model.hpp"
#else
#include "urdf/model.h"
#endif

namespace nav2_dynamic_footprint
{
namespace
{
void validate_tuning(double rate, double age)
{
  if (!std::isfinite(rate) || rate <= 0.0 || rate > 1000.0 ||
    !std::isfinite(age) || age <= 0.0)
  {
    throw std::invalid_argument("publish_rate must be in (0, 1000] and max_transform_age > 0");
  }
}
}  // namespace

template<typename NodeT>
FootprintNode<NodeT>::FootprintNode(const rclcpp::NodeOptions & options)
: NodeT("dynamic_footprint", options)
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  this->template declare_parameter<std::string>("base_frame", "base_link", startup);
  this->template declare_parameter<std::string>("robot_description", "", startup);
  this->template declare_parameter<std::string>(
    "robot_description_topic", "robot_description", startup);
  this->template declare_parameter<std::string>("footprint_topic", "/dynamic_footprint", startup);
  this->template declare_parameter<bool>("use_stamped_polygon", false, startup);
  this->template declare_parameter<double>("publish_rate", 10.0);
  this->template declare_parameter<double>("max_transform_age", 0.5);
  validation_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      double rate = this->get_parameter("publish_rate").as_double();
      double age = this->get_parameter("max_transform_age").as_double();
      try {
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "publish_rate") {
            rate = parameter.as_double();
          } else if (parameter.get_name() == "max_transform_age") {
            age = parameter.as_double();
          }
        }
        validate_tuning(rate, age);
      } catch (const std::exception & error) {
        result.successful = false;
        result.reason = error.what();
      }
      return result;
    });
  // Parameter events exist on Humble too and are emitted only after a successful commit.
  // Read the committed snapshot instead of applying event payloads, which may be older.
  parameter_events_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    [this](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event) {
      if (event->node != this->get_node_base_interface()->get_fully_qualified_name()) {
        return;
      }
      const auto tuning_changed = [](const auto & parameters) {
        return std::any_of(
          parameters.begin(), parameters.end(), [](const auto & parameter) {
            return parameter.name == "publish_rate" || parameter.name == "max_transform_age";
          });
      };
      if (!tuning_changed(event->new_parameters) && !tuning_changed(event->changed_parameters)) {
        return;
      }
      const auto parameters = this->get_parameters({"publish_rate", "max_transform_age"});
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      const bool rate_changed = publish_rate_ != parameters[0].as_double();
      publish_rate_ = parameters[0].as_double();
      max_transform_age_ = parameters[1].as_double();
      if (active_ && rate_changed) {
        reset_timer();
      }
    });
}

template<typename NodeT>
void FootprintNode<NodeT>::configure_runtime()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  base_frame_ = this->get_parameter("base_frame").as_string();
  const auto description = this->get_parameter("robot_description").as_string();
  const auto description_topic = this->get_parameter("robot_description_topic").as_string();
  const auto topic = this->get_parameter("footprint_topic").as_string();
  use_stamped_polygon_ = this->get_parameter("use_stamped_polygon").as_bool();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  max_transform_age_ = this->get_parameter("max_transform_age").as_double();
  validate_tuning(publish_rate_, max_transform_age_);
  if (base_frame_.empty() || topic.empty() || description_topic.empty()) {
    throw std::invalid_argument("Frame and topic names must not be empty");
  }
  buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
  if (use_stamped_polygon_) {
    stamped_publisher_ = this->template create_publisher<geometry_msgs::msg::PolygonStamped>(
      topic, rclcpp::QoS(1));
  } else {
    publisher_ = this->template create_publisher<geometry_msgs::msg::Polygon>(
      topic,
      rclcpp::QoS(1));
  }
  if (!description.empty()) {
    load_description(description);
  } else {
    description_subscription_ = this->template create_subscription<std_msgs::msg::String>(
      description_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        try {
          load_description(msg->data);
          RCLCPP_INFO(this->get_logger(), "Loaded robot_description from topic");
        } catch (const std::exception & error) {
          collisions_.clear();
          RCLCPP_ERROR(
            this->get_logger(), "Footprint withheld: invalid robot_description: %s",
            error.what());
        }
      });
    RCLCPP_INFO(
      this->get_logger(), "Waiting for robot_description on %s",
      description_topic.c_str());
  }
}

template<typename NodeT>
void FootprintNode<NodeT>::load_description(const std::string & description)
{
  std::vector<Collision> collisions;
  urdf::Model model;
  if (!model.initString(description) || !model.getLink(base_frame_)) {
    throw std::invalid_argument("robot_description must contain base_frame");
  }
  for (const auto & entry : model.links_) {
    for (const auto & collision : entry.second->collision_array) {
      Collision output;
      output.frame = entry.first;
      const auto & pose = collision->origin;
      const tf2::Transform origin(
        tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
      const auto & geometry = collision->geometry;
      if (!geometry) {
        throw std::invalid_argument("Collision has no geometry: " + entry.first);
      }
      double x = 0.0, y = 0.0, z = 0.0;
      if (geometry->type == urdf::Geometry::BOX) {
        const auto box = std::static_pointer_cast<urdf::Box>(geometry);
        x = box->dim.x / 2.0;
        y = box->dim.y / 2.0;
        z = box->dim.z / 2.0;
      } else if (geometry->type == urdf::Geometry::SPHERE) {
        const auto sphere = std::static_pointer_cast<urdf::Sphere>(geometry);
        x = y = z = sphere->radius;
      } else if (geometry->type == urdf::Geometry::CYLINDER) {
        const auto cylinder = std::static_pointer_cast<urdf::Cylinder>(geometry);
        x = y = cylinder->radius;
        z = cylinder->length / 2.0;
      } else if (geometry->type == urdf::Geometry::MESH) {
        const auto mesh = std::static_pointer_cast<urdf::Mesh>(geometry);
        std::unique_ptr<shapes::Mesh> shape(shapes::createMeshFromResource(
            mesh->filename, Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z)));
        if (!shape || shape->vertex_count == 0) {
          throw std::invalid_argument("Cannot load collision mesh: " + mesh->filename);
        }
        for (unsigned int i = 0; i < shape->vertex_count; ++i) {
          output.vertices.push_back(
            origin * tf2::Vector3(
              shape->vertices[3 * i], shape->vertices[3 * i + 1], shape->vertices[3 * i + 2]));
        }
      } else {
        throw std::invalid_argument("Unsupported collision geometry");
      }
      if (geometry->type != urdf::Geometry::MESH) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          x <= 0.0 || y <= 0.0 || z <= 0.0)
        {
          throw std::invalid_argument("Collision dimensions must be positive and finite");
        }
        // Bounding prisms conservatively contain curved primitives at every orientation.
        for (const double sx : {-x, x}) {
          for (const double sy : {-y, y}) {
            for (const double sz : {-z, z}) {
              output.vertices.push_back(origin * tf2::Vector3(sx, sy, sz));
            }
          }
        }
      }
      collisions.push_back(std::move(output));
    }
  }
  if (collisions.empty()) {
    throw std::invalid_argument("URDF has no collision geometry");
  }
  collisions_ = std::move(collisions);
}

template<typename NodeT>
void FootprintNode<NodeT>::update()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!active_ || collisions_.empty()) {
    return;
  }
  try {
    std::vector<Point> points;
    // Query cached transforms directly; no timeout or dedicated listener thread is needed.
    const tf2::BufferCore & transforms = *buffer_;
    // A common timestamp prevents mixing independently moving links at different times.
    const auto stamp = this->now();
    auto query = stamp;
    bool dynamic_transform = false;
    for (const auto & collision : collisions_) {
      if (collision.frame == base_frame_) {
        continue;
      }
      const auto latest = transforms.lookupTransform(
        base_frame_, collision.frame, tf2::TimePointZero);
      const rclcpp::Time latest_stamp(latest.header.stamp, this->get_clock()->get_clock_type());
      // TF returns zero for entirely static chains, which never expire.
      if (latest_stamp.nanoseconds() != 0) {
        const double age = (stamp - latest_stamp).seconds();
        if (age < 0.0) {
          throw std::runtime_error(
                  "Collision TF for " + collision.frame + " is " +
                  std::to_string(-age) + " s ahead of the node clock; check clock synchronization "
                  "and use_sim_time");
        }
        if (age > max_transform_age_) {
          throw std::runtime_error(
                  "Collision TF for " + collision.frame + " is " +
                  std::to_string(age) + " s old; max_transform_age=" +
                  std::to_string(max_transform_age_) + " s");
        }
        if (!dynamic_transform || latest_stamp < query) {
          query = latest_stamp;
        }
        dynamic_transform = true;
      }
    }
    const auto query_time = tf2::TimePoint(std::chrono::nanoseconds(query.nanoseconds()));
    for (const auto & collision : collisions_) {
      tf2::Transform transform = tf2::Transform::getIdentity();
      if (collision.frame != base_frame_) {
        const auto msg = transforms.lookupTransform(base_frame_, collision.frame, query_time);
        const auto & q = msg.transform.rotation;
        const auto & t = msg.transform.translation;
        transform = tf2::Transform(
          tf2::Quaternion(q.x, q.y, q.z, q.w),
          tf2::Vector3(t.x, t.y, t.z));
      }
      for (const auto & vertex : collision.vertices) {
        const auto p = transform * vertex;
        points.push_back({p.x(), p.y()});
      }
    }
    const auto hull = convex_hull(std::move(points));
    geometry_msgs::msg::Polygon polygon;
    for (const auto & point : hull) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(point.x);
      p.y = static_cast<float>(point.y);
      polygon.points.push_back(p);
    }
    if (use_stamped_polygon_) {
      auto message = std::make_unique<geometry_msgs::msg::PolygonStamped>();
      message->header.frame_id = base_frame_;
      message->header.stamp = query;
      message->polygon = std::move(polygon);
      stamped_publisher_->publish(std::move(message));
    } else {
      publisher_->publish(std::make_unique<geometry_msgs::msg::Polygon>(std::move(polygon)));
    }
    report_success();
  } catch (const std::exception & error) {
    report_failure(error.what());
  }
}

template<typename NodeT>
void FootprintNode<NodeT>::report_failure(const std::string & reason)
{
  const auto now = std::chrono::steady_clock::now();
  if (!publication_blocked_) {
    publication_blocked_ = true;
    blocked_since_ = now;
    // This grace period affects logging only: invalid footprints are always withheld.
    next_warning_ = now + std::chrono::seconds(published_once_ ? 5 : 2);
    if (published_once_) {
      RCLCPP_WARN(this->get_logger(), "Footprint publishing interrupted: %s", reason.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Waiting for first valid footprint: %s", reason.c_str());
    }
  } else if (now >= next_warning_) {
    const double duration = std::chrono::duration<double>(now - blocked_since_).count();
    RCLCPP_WARN(
      this->get_logger(), "Footprint still withheld after %.1f s: %s",
      duration, reason.c_str());
    next_warning_ = now + std::chrono::seconds(5);
  }
}

template<typename NodeT>
void FootprintNode<NodeT>::report_success()
{
  if (publication_blocked_) {
    const double duration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - blocked_since_).count();
    RCLCPP_INFO(
      this->get_logger(), "Footprint publishing %s after %.2f s; geometry and TF valid",
      published_once_ ? "resumed" : "started", duration);
  } else if (!published_once_) {
    RCLCPP_INFO(this->get_logger(), "Footprint publishing started; geometry and TF valid");
  }
  published_once_ = true;
  publication_blocked_ = false;
}

template<typename NodeT>
void FootprintNode<NodeT>::reset_timer()
{
  if (timer_) {
    timer_->cancel();
  }
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_), [this]() {update();});
}

template<typename NodeT>
void FootprintNode<NodeT>::activate_runtime()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if constexpr (std::is_same_v<NodeT, rclcpp_lifecycle::LifecycleNode>) {
    if (stamped_publisher_) {
      stamped_publisher_->on_activate();
    } else {
      publisher_->on_activate();
    }
  }
  published_once_ = false;
  publication_blocked_ = false;
  active_ = true;
  reset_timer();
}

template<typename NodeT>
void FootprintNode<NodeT>::deactivate_runtime()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  active_ = false;
  timer_.reset();
  if constexpr (std::is_same_v<NodeT, rclcpp_lifecycle::LifecycleNode>) {
    if (publisher_) {
      publisher_->on_deactivate();
    }
    if (stamped_publisher_) {
      stamped_publisher_->on_deactivate();
    }
  }
}

template<typename NodeT>
void FootprintNode<NodeT>::cleanup_runtime()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  deactivate_runtime();
  description_subscription_.reset();
  listener_.reset();
  buffer_.reset();
  publisher_.reset();
  stamped_publisher_.reset();
  collisions_.clear();
}

template class FootprintNode<rclcpp::Node>;
template class FootprintNode<rclcpp_lifecycle::LifecycleNode>;

DynamicFootprint::DynamicFootprint(const rclcpp::NodeOptions & options)
: FootprintNode<rclcpp::Node>(options)
{
  configure_runtime();
  activate_runtime();
}

DynamicFootprintLifecycle::DynamicFootprintLifecycle(const rclcpp::NodeOptions & options)
: FootprintNode<rclcpp_lifecycle::LifecycleNode>(options)
{
  preshutdown_callback_ = std::make_unique<rclcpp::PreShutdownCallbackHandle>(
    get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {
        on_cleanup(get_current_state());
      }));
}

DynamicFootprintLifecycle::~DynamicFootprintLifecycle()
{
  get_node_base_interface()->get_context()->remove_pre_shutdown_callback(*preshutdown_callback_);
  cleanup_runtime();
  bond_.reset();
}

DynamicFootprintLifecycle::CallbackReturn DynamicFootprintLifecycle::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
  try {
    configure_runtime();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", error.what());
    cleanup_runtime();
    return CallbackReturn::FAILURE;
  }
}

DynamicFootprintLifecycle::CallbackReturn DynamicFootprintLifecycle::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
  try {
    bond_ = std::make_shared<bond::Bond>(
      "bond", get_name(), get_node_base_interface(), get_node_logging_interface(),
      get_node_parameters_interface(), get_node_timers_interface(), get_node_topics_interface());
    bond_->setHeartbeatPeriod(0.1);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
    activate_runtime();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Activation failed: %s", error.what());
    deactivate_runtime();
    bond_.reset();
    return CallbackReturn::FAILURE;
  }
}

DynamicFootprintLifecycle::CallbackReturn DynamicFootprintLifecycle::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
  deactivate_runtime();
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

DynamicFootprintLifecycle::CallbackReturn DynamicFootprintLifecycle::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(lifecycle_mutex_);
  cleanup_runtime();
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

DynamicFootprintLifecycle::CallbackReturn DynamicFootprintLifecycle::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

DynamicFootprintLifecycle::CallbackReturn DynamicFootprintLifecycle::on_error(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}
}  // namespace nav2_dynamic_footprint

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_dynamic_footprint::DynamicFootprint)
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_dynamic_footprint::DynamicFootprintLifecycle)
