// Tracks pedestrian detections over time and classifies them as moving or not.
//
// See tracker_core.hpp for what this does and does not buy. In short: track
// confirmation does NOT filter static structure, because a rack upright is the
// most confirmable object in a warehouse. The velocity estimate is what
// separates them, and a person standing still remains genuinely ambiguous on a
// single scan plane.

#include <memory>
#include <vector>

#include "amr_perception/tracker_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace amr_perception
{

class PeopleTracker : public rclcpp::Node
{
public:
  PeopleTracker()
  : Node("people_tracker")
  {
    TrackerParams p;
    p.accel_noise = declare_parameter("accel_noise", p.accel_noise);
    p.measurement_noise = declare_parameter("measurement_noise", p.measurement_noise);
    p.gate_mahalanobis = declare_parameter("gate_mahalanobis", p.gate_mahalanobis);
    p.confirm_hits = static_cast<std::size_t>(
      declare_parameter("confirm_hits", static_cast<int>(p.confirm_hits)));
    p.confirm_window = static_cast<std::size_t>(
      declare_parameter("confirm_window", static_cast<int>(p.confirm_window)));
    p.max_misses = static_cast<std::size_t>(
      declare_parameter("max_misses", static_cast<int>(p.max_misses)));
    p.moving_speed = declare_parameter("moving_speed", p.moving_speed);
    p.moving_hold = declare_parameter("moving_hold", p.moving_hold);
    moving_only_ = declare_parameter("publish_moving_only", true);

    tracker_ = std::make_unique<MultiObjectTracker>(p);

    sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      "people_detections", rclcpp::SensorDataQoS(),
      [this](vision_msgs::msg::Detection3DArray::SharedPtr m) {onDetections(*m);});
    pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("people_tracks", 10);

    RCLCPP_INFO(
      get_logger(), "people_tracker: confirm %zu of %zu, coast %zu, moving above %.2f m/s%s",
      p.confirm_hits, p.confirm_window, p.max_misses, p.moving_speed,
      moving_only_ ? ", publishing moving tracks only" : ", publishing all confirmed tracks");
  }

private:
  void onDetections(const vision_msgs::msg::Detection3DArray & msg)
  {
    const rclcpp::Time stamp(msg.header.stamp);
    double dt = 1.0 / 14.28;
    if (have_last_) {
      const double measured = (stamp - last_stamp_).seconds();
      // Guard against a clock jump or a replayed bag rewinding time.
      if (measured > 1e-4 && measured < 1.0) {dt = measured;}
    }
    last_stamp_ = stamp;
    have_last_ = true;

    std::vector<Observation> obs;
    obs.reserve(msg.detections.size());
    for (const auto & d : msg.detections) {
      obs.push_back(
        Observation{d.bbox.center.position.x, d.bbox.center.position.y,
          d.results.empty() ? 1.0 : d.results[0].hypothesis.score});
    }

    tracker_->update(obs, dt);

    vision_msgs::msg::Detection3DArray out;
    out.header = msg.header;
    std::size_t moving = 0;
    for (const auto & t : tracker_->confirmedTracks()) {
      const bool is_moving = t.isMoving(tracker_->params());
      if (is_moving) {++moving;}
      if (moving_only_ && !is_moving) {continue;}

      vision_msgs::msg::Detection3D det;
      det.header = msg.header;
      det.id = std::to_string(t.id);
      det.bbox.center.position.x = t.x[0];
      det.bbox.center.position.y = t.x[1];
      det.bbox.center.orientation.w = 1.0;
      det.bbox.size.x = 0.35;
      det.bbox.size.y = 0.35;
      det.bbox.size.z = 1.75;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = is_moving ? "person_moving" : "person_static";
      hyp.hypothesis.score = is_moving ? 0.9 : 0.4;
      hyp.pose.pose = det.bbox.center;
      // Velocity travels in the covariance slot because Detection3D has no
      // twist field. Documented here rather than left for a reader to guess.
      hyp.pose.covariance[0] = t.x[2];
      hyp.pose.covariance[7] = t.x[3];
      det.results.push_back(hyp);
      out.detections.push_back(det);
    }
    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "%zu detections -> %zu tracks (%zu confirmed, %zu moving), published %zu",
      obs.size(), tracker_->tracks().size(),
      tracker_->confirmedTracks().size(), moving, out.detections.size());
  }

  std::unique_ptr<MultiObjectTracker> tracker_;
  bool moving_only_{true};
  bool have_last_{false};
  rclcpp::Time last_stamp_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
};

}  // namespace amr_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_perception::PeopleTracker>());
  rclcpp::shutdown();
  return 0;
}
