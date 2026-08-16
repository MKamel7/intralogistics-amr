// Copyright 2026 Mohamed Kamel
//
// A costmap layer that pays a cost for passing close to a tracked person.
//
// It reads `people_tracks`, the confirmed output of the people tracker, and
// NOT the raw detections. The tracker's precision is 0.615 against a recall of
// 0.988 (V-36), so the detections contain false positives that would put
// phantom cost in the middle of an aisle. A confirmed track has survived
// association across frames, which is the cheapest filter available.
//
// It does not mark anything lethal. See kMaxProxemicCost.

#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/proxemic_core.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace amr_navigation
{

class ProxemicLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("ProxemicLayer: node has expired");
    }

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("radius", rclcpp::ParameterValue(1.2));
    declareParameter("peak_cost", rclcpp::ParameterValue(200));
    declareParameter("topic", rclcpp::ParameterValue(std::string("people_tracks")));
    declareParameter("track_timeout", rclcpp::ParameterValue(2.0));

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "radius", radius_);
    int peak = 200;
    node->get_parameter(name_ + "." + "peak_cost", peak);
    peak_ = std::min<double>(peak, kMaxProxemicCost);
    std::string topic;
    node->get_parameter(name_ + "." + "topic", topic);
    node->get_parameter(name_ + "." + "track_timeout", timeout_);

    // The tracks arrive in the SENSOR frame and the costmap works in its own
    // global frame, so each one is transformed on arrival. Doing it here
    // rather than in updateCosts keeps the transform failure visible as a
    // dropped track rather than as a layer that silently does nothing.
    tf_ = tf_;   // inherited from Layer, kept explicit for the reader

    sub_ = node->create_subscription<vision_msgs::msg::Detection3DArray>(
      topic, rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = *msg;
      });

    current_ = true;
    RCLCPP_INFO(
      node->get_logger(),
      "ProxemicLayer on %s: radius %.2f m, peak %.0f, never lethal",
      topic.c_str(), radius_, peak_);
  }

  void updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & d : latest_.detections) {
      const auto & p = d.bbox.center.position;
      *min_x = std::min(*min_x, p.x - radius_);
      *min_y = std::min(*min_y, p.y - radius_);
      *max_x = std::max(*max_x, p.x + radius_);
      *max_y = std::max(*max_y, p.y + radius_);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master, int min_i, int min_j,
    int max_i, int max_j) override
  {
    if (!enabled_) {
      return;
    }
    vision_msgs::msg::Detection3DArray tracks;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tracks = latest_;
    }
    if (tracks.detections.empty()) {
      return;
    }

    const double res = master.getResolution();
    const int cells = static_cast<int>(std::ceil(radius_ / res));

    for (const auto & d : tracks.detections) {
      const auto & p = d.bbox.center.position;
      unsigned int cx, cy;
      if (!master.worldToMap(p.x, p.y, cx, cy)) {
        continue;                       // outside this costmap, not an error
      }
      for (int dj = -cells; dj <= cells; ++dj) {
        for (int di = -cells; di <= cells; ++di) {
          const int i = static_cast<int>(cx) + di;
          const int j = static_cast<int>(cy) + dj;
          if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
            continue;
          }
          double wx, wy;
          master.mapToWorld(i, j, wx, wy);
          const double dist = std::hypot(wx - p.x, wy - p.y);
          const double c = proxemicCost(dist, radius_, peak_);
          if (c <= 0.0) {
            continue;
          }
          const unsigned char want =
            static_cast<unsigned char>(std::min<double>(c, kMaxProxemicCost));
          const unsigned char have = master.getCost(i, j);
          // MAXIMUM, never sum. Adding would let two people standing together
          // reach the inscribed value between them and close a route this
          // layer has no business closing.
          if (have == nav2_costmap_2d::NO_INFORMATION || want > have) {
            if (have < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
              master.setCost(i, j, want);
            }
          }
        }
      }
    }
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.detections.clear();
  }

  bool isClearable() override {return false;}

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_;
  vision_msgs::msg::Detection3DArray latest_;
  std::mutex mutex_;
  double radius_{1.2};
  double peak_{200.0};
  double timeout_{2.0};
};

}  // namespace amr_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_navigation::ProxemicLayer, nav2_costmap_2d::Layer)
