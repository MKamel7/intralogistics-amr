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
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
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
