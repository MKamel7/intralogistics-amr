// Copyright 2026 Mohamed Kamel
//
// Publishes the pose of a V shaped dock, found in the merged scan.
//
// PERCEPTION ONLY. This node steers nothing. It answers "where is the dock,
// relative to the vehicle" and the closing controller lives in the mission
// layer, because a node that both looks and drives cannot be tested by feeding
// it scans.
//
// WHY THE MERGED SCAN AND NOT ONE SCANNER
//
// The dock is approached head on and the merged scan is the only topic where a
// forward sector exists as one coherent sweep: the two corner scanners each see
// part of it, from different origins, and stitching them per detection would be
// the scan merger again with fewer tests.
//
// The merged scan has already had the vehicle's own body removed by the shaped
// self filter, so the returns arriving here are the world. That matters: a
// self return inside the search window is a surface 0.29 m away at a plausible
// angle, which is exactly what this detector is looking for.

#include <memory>
#include <string>
#include <vector>

#include "amr_perception/dock_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_perception
{

class DockDetector : public rclcpp::Node
{
public:
  DockDetector()
  : Node("dock_detector")
  {
    spec_.opening = declare_parameter("opening", spec_.opening);
    spec_.opening_tol = declare_parameter("opening_tol", spec_.opening_tol);
    spec_.min_range = declare_parameter("min_range", spec_.min_range);
    spec_.max_range = declare_parameter("max_range", spec_.max_range);
    spec_.half_sector = declare_parameter("half_sector", spec_.half_sector);
    spec_.max_residual = declare_parameter("max_residual", spec_.max_residual);
    spec_.min_face = declare_parameter("min_face", spec_.min_face);
    spec_.min_points = static_cast<std::size_t>(
      declare_parameter("min_points", static_cast<int>(spec_.min_points)));

    // The gate. Commissioning data, like the keepout zones: where the dock is
    // comes from the site layout, not from perception. See DockGate.
    gate_.enabled = declare_parameter("gate_enabled", gate_.enabled);
    gate_.x = declare_parameter("dock_x", gate_.x);
    gate_.y = declare_parameter("dock_y", gate_.y);
    gate_.radius = declare_parameter("gate_radius", gate_.radius);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 10);
    // A SEPARATE, EXPLICIT "no dock" SIGNAL. A controller that infers absence
    // from a stale pose will drive at a dock that is no longer there, and a
    // topic that simply stops publishing looks identical to a node that died.
    found_pub_ = create_publisher<std_msgs::msg::Bool>("dock_found", 10);

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&DockDetector::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "dock detector: opening %.1f deg +/- %.1f, range %.2f to %.2f m, "
      "sector +/- %.1f deg, faces at least %.2f m",
      spec_.opening * 180.0 / M_PI, spec_.opening_tol * 180.0 / M_PI,
      spec_.min_range, spec_.max_range, spec_.half_sector * 180.0 / M_PI,
      spec_.min_face);
    if (gate_.enabled) {
      RCLCPP_INFO(
        get_logger(), "gated: searching only within %.1f m of the dock at (%.2f, %.2f)",
        gate_.radius, gate_.x, gate_.y);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "GATE DISABLED: searching every scan. V-66 measured 84 percent false "
        "positives this way");
    }
  }

private:
  /// The vehicle's own belief about where it is, from localisation rather than
  /// from the oracle: ADR 0006 keeps ground truth out of the control path, and
  /// a gate that decides whether to steer is in it.
  bool vehiclePose(double & x, double & y) const
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double vx = 0.0, vy = 0.0;
    const bool have_pose = vehiclePose(vx, vy);
    if (!shouldSearch(gate_, have_pose, vx, vy)) {
      // Say "no dock" rather than going quiet: a controller that infers absence
      // from silence cannot tell a gated detector from a dead one, which is the
      // same argument the found topic already exists for.
      std_msgs::msg::Bool gated;
      gated.data = false;
      found_pub_->publish(gated);
      return;
    }

    std::vector<Point2> pts;
    pts.reserve(msg->ranges.size());
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      pts.push_back({r * std::cos(a), r * std::sin(a)});
    }

    const auto dock = findDock(pts, spec_);

    std_msgs::msg::Bool found;
    found.data = dock.has_value();
    found_pub_->publish(found);

    if (!dock) {
      // Counted rather than logged per scan, because at 14 Hz a warning per
      // miss is a log nobody can read and the interesting figure is the rate.
      ++misses_;
      if (++since_report_ >= 140) {          // about ten seconds
        RCLCPP_INFO(
          get_logger(), "dock: %zu found, %zu not found in the last %zu scans",
          hits_, misses_, hits_ + misses_);
        hits_ = misses_ = since_report_ = 0;
      }
      return;
    }
    ++hits_;
    ++since_report_;

    geometry_msgs::msg::PoseStamped out;
    out.header = msg->header;               // the scan's frame and stamp, not now
    out.pose.position.x = dock->x;
    out.pose.position.y = dock->y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, dock->yaw);
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
    pose_pub_->publish(out);
  }

  DockSpec spec_;
  DockGate gate_;
  std::string map_frame_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr found_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::size_t hits_{0}, misses_{0}, since_report_{0};
};

}  // namespace amr_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_perception::DockDetector>());
  rclcpp::shutdown();
  return 0;
}
