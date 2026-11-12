#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "nav2_dynamic_footprint/dynamic_footprint.hpp"

using namespace std::chrono_literals;

class Collisions : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static rclcpp::NodeOptions options(const std::string & collisions)
  {
    rclcpp::NodeOptions result;
    result.parameter_overrides(
      {rclcpp::Parameter(
          "robot_description",
          "<robot name='test'><link name='base_link'>" + collisions + "</link></robot>")});
    return result;
  }

  void expectBounds(
    const std::string & geometry, const std::string & origin,
    double min_x, double max_x, double min_y, double max_y)
  {
    auto node = std::make_shared<nav2_dynamic_footprint::DynamicFootprint>(
      options(
        "<collision>" + origin + "<geometry>" + geometry + "</geometry></collision>"));
    auto observer = std::make_shared<rclcpp::Node>("collision_observer");
    geometry_msgs::msg::Polygon::ConstSharedPtr received;
    auto subscription = observer->create_subscription<geometry_msgs::msg::Polygon>(
      "/dynamic_footprint", 1, [&received](geometry_msgs::msg::Polygon::ConstSharedPtr msg) {
        received = std::move(msg);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(observer);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(20ms);
    }
    ASSERT_TRUE(received);
    double x0 = std::numeric_limits<double>::infinity(), y0 = x0;
    double x1 = -x0, y1 = -y0;
    for (const auto & p : received->points) {
      x0 = std::min(x0, static_cast<double>(p.x));
      x1 = std::max(x1, static_cast<double>(p.x));
      y0 = std::min(y0, static_cast<double>(p.y));
      y1 = std::max(y1, static_cast<double>(p.y));
    }
    EXPECT_NEAR(x0, min_x, 1e-5);
    EXPECT_NEAR(x1, max_x, 1e-5);
    EXPECT_NEAR(y0, min_y, 1e-5);
    EXPECT_NEAR(y1, max_y, 1e-5);
  }
};

TEST_F(Collisions, RotatedOffsetBox)
{
  expectBounds(
    "<box size='2 1 1'/>", "<origin xyz='1 2 0' rpy='0 0 1.5707963267948966'/>",
    0.5, 1.5, 1, 3);
}

TEST_F(Collisions, CurvedPrimitivesConservativelyBounded)
{
  expectBounds("<sphere radius='1'/>", "", -1, 1, -1, 1);
  expectBounds(
    "<cylinder radius='1' length='4'/>",
    "<origin rpy='1.5707963267948966 0 0'/>", -1, 1, -2, 2);
}

TEST_F(Collisions, ScaledMeshResource)
{
  expectBounds(
    "<mesh filename='file://" TEST_MESH "' scale='2 3 1'/>",
    "<origin xyz='1 2 0'/>", 1, 3, 2, 5);
}

TEST_F(Collisions, InvalidDescriptionAndEmptyCollisionsFailConstruction)
{
  rclcpp::NodeOptions invalid;
  invalid.parameter_overrides({rclcpp::Parameter("robot_description", "invalid XML")});
  EXPECT_THROW(
    std::make_shared<nav2_dynamic_footprint::DynamicFootprint>(invalid),
    std::invalid_argument);
  EXPECT_THROW(
    std::make_shared<nav2_dynamic_footprint::DynamicFootprint>(options("")),
    std::invalid_argument);
  EXPECT_THROW(
    std::make_shared<nav2_dynamic_footprint::DynamicFootprint>(
      options(
        "<collision><geometry><box size='0 1 1'/></geometry></collision>")),
    std::invalid_argument);
}

TEST_F(Collisions, RetainedTopicDescriptionAndUpdates)
{
  auto source = std::make_shared<rclcpp::Node>("description_source");
  auto publisher = source->create_publisher<std_msgs::msg::String>(
    "test_description", rclcpp::QoS(1).transient_local());
  std_msgs::msg::String description;
  const auto xml = [](const std::string & size) {
      return "<robot name='test'><link name='base_link'><collision><geometry><box size='" +
             size + "'/></geometry></collision></link></robot>";
    };
  description.data = xml("2 1 1");
  // Publish before the subscriber exists to verify transient-local delivery.
  publisher->publish(description);
  rclcpp::NodeOptions topic_options;
  topic_options.parameter_overrides(
  {
    rclcpp::Parameter("robot_description_topic", "test_description"),
    rclcpp::Parameter("footprint_topic", "test_topic_footprint")});
  auto node = std::make_shared<nav2_dynamic_footprint::DynamicFootprint>(topic_options);
  geometry_msgs::msg::Polygon::ConstSharedPtr received;
  size_t count = 0;
  auto subscription = source->create_subscription<geometry_msgs::msg::Polygon>(
    "test_topic_footprint", 1, [&](geometry_msgs::msg::Polygon::ConstSharedPtr msg) {
      received = std::move(msg);
      ++count;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(source);
  const auto wait = [&executor](const auto & predicate) {
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_once(20ms);
        if (predicate()) {
          return true;
        }
      }
      return false;
    };
  const auto max_x = [&received]() {
      double result = -std::numeric_limits<double>::infinity();
      if (received) {
        for (const auto & p : received->points) {
          result = std::max(result, static_cast<double>(p.x));
        }
      }
      return result;
    };
  ASSERT_TRUE(wait([&]() {return max_x() == 1.0;}));
  description.data = xml("4 1 1");
  publisher->publish(description);
  ASSERT_TRUE(wait([&]() {return max_x() == 2.0;}));
  description.data = "invalid URDF";
  publisher->publish(description);
  const auto drain_until = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < drain_until) {
    executor.spin_once(20ms);
  }
  const auto before = count;
  const auto stop_until = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < stop_until) {
    executor.spin_once(20ms);
  }
  EXPECT_EQ(count, before);
  description.data = xml("6 1 1");
  publisher->publish(description);
  ASSERT_TRUE(wait([&]() {return max_x() == 3.0;}));
}

TEST_F(Collisions, ParameterDescriptionDisablesTopicSubscription)
{
  auto node = std::make_shared<nav2_dynamic_footprint::DynamicFootprint>(
    options(
      "<collision><geometry><box size='1 1 1'/></geometry></collision>"));
  for (const auto & endpoint : node->get_subscriptions_info_by_topic("robot_description")) {
    EXPECT_NE(endpoint.node_name(), node->get_name());
  }
}
