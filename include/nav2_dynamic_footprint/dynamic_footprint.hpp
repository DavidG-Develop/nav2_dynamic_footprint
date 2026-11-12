#ifndef NAV2_DYNAMIC_FOOTPRINT__DYNAMIC_FOOTPRINT_HPP_
#define NAV2_DYNAMIC_FOOTPRINT__DYNAMIC_FOOTPRINT_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include "bondcpp/bond.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace nav2_dynamic_footprint
{
// Shared implementation for plain and managed component entry points.
template<typename NodeT>
class FootprintNode : public NodeT
{
protected:
  explicit FootprintNode(const rclcpp::NodeOptions & options);
  void configure_runtime();
  void activate_runtime();
  void deactivate_runtime();
  void cleanup_runtime();

private:
  struct Collision
  {
    std::string frame;
    std::vector<tf2::Vector3> vertices;
  };
  template<typename MessageT>
  using Publisher = std::conditional_t<
    std::is_same_v<NodeT, rclcpp_lifecycle::LifecycleNode>,
    rclcpp_lifecycle::LifecyclePublisher<MessageT>,
    rclcpp::Publisher<MessageT>>;
  void load_description(const std::string & description);
  void update();
  void reset_timer();
  void report_failure(const std::string & reason);
  void report_success();
  std::recursive_mutex mutex_;
  bool active_{false};
  bool published_once_{false};
  bool publication_blocked_{false};
  std::chrono::steady_clock::time_point blocked_since_;
  std::chrono::steady_clock::time_point next_warning_;
  bool use_stamped_polygon_{false};
  std::string base_frame_;
  double publish_rate_{10.0};
  double max_transform_age_{0.5};
  std::vector<Collision> collisions_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  typename Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
  typename Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr stamped_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_callback_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr update_callback_;
};

class DynamicFootprint : public FootprintNode<rclcpp::Node>
{
public:
  explicit DynamicFootprint(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

class DynamicFootprintLifecycle : public FootprintNode<rclcpp_lifecycle::LifecycleNode>
{
public:
  explicit DynamicFootprintLifecycle(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DynamicFootprintLifecycle() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  std::recursive_mutex lifecycle_mutex_;
  std::shared_ptr<bond::Bond> bond_;
  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> preshutdown_callback_;
};
}  // namespace nav2_dynamic_footprint
#endif  // NAV2_DYNAMIC_FOOTPRINT__DYNAMIC_FOOTPRINT_HPP_
