// Merges the two safety scanners into a single 360 degree scan in base_link.
//
// Motion compensation, and why it is not optional here. The two scanners run at
// 14.28 Hz and are not phase locked, so their samples can be up to half a period
// apart, about 35 ms. At the platform's 2.0 m/s top speed that is 70 mm of
// travel, which is larger than the 20 mm object the scanner is specified to
// detect. Merging both scans as though they were simultaneous would therefore
// smear the world by more than the sensor's own resolution.
//
// So each scan is transformed using the transform that held AT ITS OWN
// TIMESTAMP, into the odometry frame, and then into base_link as of the output
// timestamp. TF does the interpolation; the node just has to ask for the right
// times, which is the part that is easy to get wrong and invisible when it is.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "amr_perception/scan_merge_core.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_perception
{

class ScanMerger : public rclcpp::Node
{
public:
  ScanMerger()
  : Node("scan_merger")
  {
    front_topic_ = declare_parameter("front_topic", std::string("scanner_front_left/scan"));
    rear_topic_ = declare_parameter("rear_topic", std::string("scanner_rear_right/scan"));
    output_topic_ = declare_parameter("output_topic", std::string("scan"));
    target_frame_ = declare_parameter("target_frame", std::string("base_link"));
    fixed_frame_ = declare_parameter("fixed_frame", std::string("odom"));

    bins_ = static_cast<std::size_t>(declare_parameter("bins", 2118));
    range_min_ = declare_parameter("range_min", 0.05);
    range_max_ = declare_parameter("range_max", 40.0);
    publish_rate_ = declare_parameter("publish_rate", 14.28);
    // Older than this and a scan is not worth merging; better a gap than a
    // stale return presented as current.
    max_age_ = declare_parameter("max_scan_age", 0.20);

    const double footprint_length = declare_parameter("footprint_length", 0.800);
    const double footprint_width = declare_parameter("footprint_width", 0.580);
    const double footprint_margin = declare_parameter("footprint_margin", 0.020);
    footprint_ = std::make_unique<FootprintFilter>(
      footprint_length, footprint_width, footprint_margin);

    accumulator_ = std::make_unique<ScanAccumulator>(bins_, range_min_, range_max_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::SensorDataQoS();
    front_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      front_topic_, qos, [this](sensor_msgs::msg::LaserScan::SharedPtr m) {front_ = m;});
    rear_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      rear_topic_, qos, [this](sensor_msgs::msg::LaserScan::SharedPtr m) {rear_ = m;});
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_), [this] {merge();});

    RCLCPP_INFO(
      get_logger(),
      "scan_merger: %s + %s -> %s in %s, %zu bins, footprint %.3f x %.3f + %.3f m",
      front_topic_.c_str(), rear_topic_.c_str(), output_topic_.c_str(),
      target_frame_.c_str(), bins_, footprint_length, footprint_width, footprint_margin);
  }

private:
  /// Transform from `source` at `stamp` into the target frame at `out_stamp`,
  /// routed through the fixed frame so vehicle motion between the two instants
  /// is accounted for rather than ignored.
  bool lookup(
    const std::string & source, const rclcpp::Time & stamp,
    const rclcpp::Time & out_stamp, Transform2D & out) const
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_, out_stamp, source, stamp, fixed_frame_,
        rclcpp::Duration::from_seconds(0.05));
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      tf2::Quaternion q(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      out.yaw = yaw;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "transform %s -> %s unavailable: %s",
        source.c_str(), target_frame_.c_str(), e.what());
      return false;
    }
  }

  void merge()
  {
    if (!front_ || !rear_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "waiting for both scanners (front:%d rear:%d)", front_ != nullptr, rear_ != nullptr);
      return;
    }

    const rclcpp::Time t_front(front_->header.stamp);
    const rclcpp::Time t_rear(rear_->header.stamp);
    // Merge as of the OLDER scan. Extrapolating the older one forward to the
    // newer would mean inventing data; waiting for the newer is honest.
    const rclcpp::Time out_stamp = (t_front < t_rear) ? t_front : t_rear;

    const double age = (now() - out_stamp).seconds();
    if (age > max_age_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "scans are %.3f s old, beyond the %.3f s limit; not publishing", age, max_age_);
      return;
    }

    Transform2D tf_front, tf_rear;
    if (!lookup(front_->header.frame_id, t_front, out_stamp, tf_front)) {return;}
    if (!lookup(rear_->header.frame_id, t_rear, out_stamp, tf_rear)) {return;}

    accumulator_->reset();

    ScanView vf{front_->angle_min, front_->angle_increment,
      front_->range_min, front_->range_max, &front_->ranges};
    ScanView vr{rear_->angle_min, rear_->angle_increment,
      rear_->range_min, rear_->range_max, &rear_->ranges};
    accumulator_->add(vf, tf_front, *footprint_);
    accumulator_->add(vr, tf_rear, *footprint_);

    sensor_msgs::msg::LaserScan out;
    out.header.stamp = out_stamp;
    out.header.frame_id = target_frame_;
    out.angle_min = static_cast<float>(accumulator_->angleMin());
    out.angle_max = static_cast<float>(accumulator_->angleMin() + 2.0 * M_PI);
    out.angle_increment = static_cast<float>(accumulator_->angleIncrement());
    out.range_min = static_cast<float>(range_min_);
    out.range_max = static_cast<float>(range_max_);
    out.scan_time = static_cast<float>(1.0 / publish_rate_);
    out.time_increment = 0.0F;
    out.ranges = accumulator_->ranges();
    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "merged %zu returns, dropped %zu self / %zu out of range / %zu invalid; "
      "%zu of %zu bins empty",
      accumulator_->accepted(), accumulator_->selfReturns(),
      accumulator_->outOfRange(), accumulator_->invalid(),
      accumulator_->emptyBins(), accumulator_->bins());
  }

  std::string front_topic_, rear_topic_, output_topic_, target_frame_, fixed_frame_;
  std::size_t bins_;
  double range_min_, range_max_, publish_rate_, max_age_;

  std::unique_ptr<FootprintFilter> footprint_;
  std::unique_ptr<ScanAccumulator> accumulator_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::LaserScan::SharedPtr front_, rear_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_, rear_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_perception::ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
