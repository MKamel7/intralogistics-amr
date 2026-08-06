// Detects people in the merged 360 degree scan and publishes their positions.
//
// The detection is geometric: cluster the plane, keep clusters that are leg
// sized and round rather than flat, pair them at a plausible stance width. All
// of that lives in leg_detector_core so it can be tested without a simulator;
// this node is plumbing and parameters.

#include <memory>
#include <string>
#include <vector>

#include "amr_perception/leg_detector_core.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace amr_perception
{

class LegDetector : public rclcpp::Node
{
public:
  LegDetector()
  : Node("leg_detector")
  {
    params_.cluster_jump_base = declare_parameter("cluster_jump_base", 0.08);
    params_.cluster_jump_slope = declare_parameter("cluster_jump_slope", 3.0);
    params_.min_points = static_cast<std::size_t>(declare_parameter("min_points", 3));
    params_.leg_width_min = declare_parameter("leg_width_min", 0.040);
    params_.leg_width_max = declare_parameter("leg_width_max", 0.250);
    params_.leg_min_depth_ratio = declare_parameter("leg_min_depth_ratio", 0.08);
    params_.pair_separation_min = declare_parameter("pair_separation_min", 0.05);
    params_.pair_separation_max = declare_parameter("pair_separation_max", 0.55);
    params_.single_leg_max_range = declare_parameter("single_leg_max_range", 4.0);
    params_.max_bridge_bins = static_cast<std::size_t>(
      declare_parameter("max_bridge_bins", 3));
    params_.blob_max_range = declare_parameter("blob_max_range", 2.20);
    params_.blob_width_min = declare_parameter("blob_width_min", 0.16);
    params_.blob_width_max = declare_parameter("blob_width_max", 0.75);

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos, [this](sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(*m);});
    det_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("people_detections", 10);
    // A plain PoseArray alongside the typed message, purely so RViz can show
    // the result without a custom display.
    viz_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("people_markers", 10);

    RCLCPP_INFO(
      get_logger(), "leg_detector: leg width %.3f to %.3f m, stance %.2f to %.2f m",
      params_.leg_width_min, params_.leg_width_max,
      params_.pair_separation_min, params_.pair_separation_max);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    const auto clusters = segmentScan(
      scan.ranges, scan.angle_min, scan.angle_increment, params_);

    const auto people = detectPeople(clusters, params_);
    std::size_t legs = 0;
    for (const auto & c : clusters) {
      if (looksLikeLeg(c, params_)) {++legs;}
    }

    vision_msgs::msg::Detection3DArray out;
    out.header = scan.header;
    geometry_msgs::msg::PoseArray viz;
    viz.header = scan.header;

    for (const auto & person : people) {
      vision_msgs::msg::Detection3D det;
      det.header = scan.header;
      det.bbox.center.position.x = person.x;
      det.bbox.center.position.y = person.y;
      det.bbox.center.orientation.w = 1.0;
      // A person's extent on a 150 mm scan plane is their stance, not their
      // shoulders. Height is not observable from one plane and is left at the
      // model's nominal rather than guessed per detection.
      det.bbox.size.x = 0.35;
      det.bbox.size.y = 0.35;
      det.bbox.size.z = 1.75;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = person.paired ? "person" : "person_single_leg";
      hyp.hypothesis.score = person.confidence;
      hyp.pose.pose = det.bbox.center;
      det.results.push_back(hyp);
      out.detections.push_back(det);

      geometry_msgs::msg::Pose p;
      p.position.x = person.x;
      p.position.y = person.y;
      p.orientation.w = 1.0;
      viz.poses.push_back(p);
    }

    det_pub_->publish(out);
    viz_pub_->publish(viz);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "%zu clusters -> %zu leg-like -> %zu people", clusters.size(), legs, people.size());
  }

  DetectorParams params_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr viz_pub_;
};

}  // namespace amr_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_perception::LegDetector>());
  rclcpp::shutdown();
  return 0;
}
