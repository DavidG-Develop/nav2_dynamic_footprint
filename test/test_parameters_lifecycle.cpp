#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_dynamic_footprint/dynamic_footprint.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using namespace std::chrono_literals;
using nav2_dynamic_footprint::DynamicFootprint;
using nav2_dynamic_footprint::DynamicFootprintLifecycle;

namespace
{
const char * description =
  "<robot name='test'><link name='base_link'><collision><geometry>"
  "<box size='1 1 1'/></geometry></collision></link></robot>";

rclcpp::NodeOptions options()
{
  rclcpp::NodeOptions result;
  result.parameter_overrides({rclcpp::Parameter("robot_description", description)});
  return result;
}

void spin_for(rclcpp::Executor & executor, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(10ms);
  }
}
}  // namespace

class ParametersLifecycle : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(ParametersLifecycle, RejectedAtomicUpdatesHaveNoSideEffects)
{
  auto node = std::make_shared<DynamicFootprint>(options());
  const std::vector<double> invalid = {0, -1, 1001,
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()};
  for (const double rate : invalid) {
    EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate",
        rate)}).successful);
  }
  for (const double age : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(node->set_parameters_atomically({
      rclcpp::Parameter("publish_rate", 20.0),
      rclcpp::Parameter("max_transform_age", age)}).successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("publish_rate").as_double(), 10.0);
    EXPECT_DOUBLE_EQ(node->get_parameter("max_transform_age").as_double(), 0.5);
  }
  for (const auto & parameter : {
    rclcpp::Parameter("base_frame", "other"),
    rclcpp::Parameter("footprint_topic", "/other"),
    rclcpp::Parameter("robot_description_topic", "/other"),
    rclcpp::Parameter("robot_description", "other")})
  {
    EXPECT_FALSE(node->set_parameters_atomically({parameter}).successful);
  }
  EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate", 20)}).successful);
}

TEST_F(ParametersLifecycle, PublishRateChangesImmediately)
{
  auto node = std::make_shared<DynamicFootprint>(options());
  auto observer = std::make_shared<rclcpp::Node>("rate_observer");
  size_t count = 0;
  auto subscription = observer->create_subscription<geometry_msgs::msg::Polygon>(
    "/dynamic_footprint", 10, [&](geometry_msgs::msg::Polygon::ConstSharedPtr) {++count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate", 2.0)}).successful);
  spin_for(executor, 1200ms);
  const auto slow = count;
  EXPECT_GE(slow, 1u);
  EXPECT_LE(slow, 3u);
  ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate",
      80.0)}).successful);
  spin_for(executor, 1200ms);
  EXPECT_GT(count - slow, 30u);
}

TEST_F(ParametersLifecycle, LifecycleTransitionsGatePublishingAndReleaseResources)
{
  auto node = std::make_shared<DynamicFootprintLifecycle>(options());
  auto observer = std::make_shared<rclcpp::Node>("lifecycle_observer");
  size_t count = 0;
  auto subscription = observer->create_subscription<geometry_msgs::msg::Polygon>(
    "/dynamic_footprint", 10, [&](geometry_msgs::msg::Polygon::ConstSharedPtr) {++count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(executor, 200ms);
  EXPECT_EQ(count, 0u);
  EXPECT_EQ(node->count_publishers("/dynamic_footprint"), 0u);
  for (int cycle = 0; cycle < 2; ++cycle) {
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const auto before = count;
    spin_for(executor, 200ms);
    EXPECT_EQ(count, before);
    ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate",
        50.0)}).successful);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    spin_for(executor, 300ms);
    EXPECT_GT(count, before);
    ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    spin_for(executor, 100ms);  // Drain samples published before the transition.
    const auto stopped = count;
    ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate",
        20.0)}).successful);
    spin_for(executor, 200ms);
    EXPECT_EQ(count, stopped);
    ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(node->count_publishers("/dynamic_footprint"), 0u);
  }
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_EQ(node->count_publishers("/dynamic_footprint"), 0u);
}

TEST_F(ParametersLifecycle, FailedConfigurationCanBeRetriedAfterTuningCorrection)
{
  auto invalid = options();
  invalid.append_parameter_override("publish_rate", -1.0);
  auto node = std::make_shared<DynamicFootprintLifecycle>(invalid);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node->count_publishers("/dynamic_footprint"), 0u);
  ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("publish_rate",
      20.0)}).successful);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ParametersLifecycle, TransformAgeChangesAffectPublication)
{
  auto config = options();
  config.append_parameter_override("robot_description", std::string(
    "<robot name='test'><link name='base_link'/><link name='threshold_link'>"
    "<collision><geometry><box size='1 1 1'/></geometry></collision></link>"
    "<joint name='test_joint' type='fixed'><parent link='base_link'/>"
    "<child link='threshold_link'/></joint></robot>"));
  config.append_parameter_override("max_transform_age", 0.1);
  auto node = std::make_shared<DynamicFootprint>(config);
  auto source = std::make_shared<rclcpp::Node>("age_test_source");
  tf2_ros::TransformBroadcaster broadcaster(*source);
  auto timer = source->create_wall_timer(20ms, [&]() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "threshold_link";
        transform.header.stamp = source->now() - rclcpp::Duration::from_seconds(0.3);
        transform.transform.rotation.w = 1.0;
        broadcaster.sendTransform(transform);
  });
  size_t count = 0;
  auto subscription = source->create_subscription<geometry_msgs::msg::Polygon>(
    "/dynamic_footprint", 10, [&](geometry_msgs::msg::Polygon::ConstSharedPtr) {++count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(source);
  spin_for(executor, 500ms);
  EXPECT_EQ(count, 0u);
  ASSERT_TRUE(node->set_parameters_atomically({
    rclcpp::Parameter("max_transform_age", 1.0)}).successful);
  spin_for(executor, 500ms);
  EXPECT_GT(count, 0u);
  ASSERT_TRUE(node->set_parameters_atomically({
    rclcpp::Parameter("max_transform_age", 0.1)}).successful);
  spin_for(executor, 100ms);
  const auto stopped = count;
  spin_for(executor, 300ms);
  EXPECT_EQ(count, stopped);
}
