#include "dora2filter/dora2_filter.hpp"

#include <cmath>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace dora2filter
{

bool Dora2Filter::configure()
{
  getParam("threshold", threshold_, true, 0.83);
  return true;
}

bool Dora2Filter::update(
  const sensor_msgs::msg::LaserScan & input_scan,
  sensor_msgs::msg::LaserScan & output_scan)
{
  output_scan = input_scan;
  const auto & in_ranges = input_scan.ranges;
  auto & ranges = output_scan.ranges;

  if (ranges.size() < 2) {
    return true;
  }

  const double angle_min = static_cast<double>(input_scan.angle_min);
  const double angle_inc = static_cast<double>(input_scan.angle_increment);

  for (size_t i = 0; i + 1 < ranges.size(); ++i) {
    const float r1 = in_ranges[i];
    const float r2 = in_ranges[i + 1];

    if (!std::isfinite(r1) || !std::isfinite(r2)) {
      continue;
    }

    const double th1 = angle_min + static_cast<double>(i) * angle_inc;
    const double th2 = angle_min + static_cast<double>(i + 1) * angle_inc;

    const double ax = static_cast<double>(r1) * std::cos(th1);
    const double ay = static_cast<double>(r1) * std::sin(th1);
    const double bx = static_cast<double>(r2) * std::cos(th2);
    const double by = static_cast<double>(r2) * std::sin(th2);

    const double vx = bx - ax;
    const double vy = by - ay;
    const double mx = (ax + bx) * 0.5;
    const double my = (ay + by) * 0.5;

    const double vnorm = std::hypot(vx, vy);
    const double mnorm = std::hypot(mx, my);
    if (vnorm == 0.0 || mnorm == 0.0) {
      continue;
    }

    const double dot = std::fabs((vx * mx + vy * my) / (vnorm * mnorm));
    if (dot > threshold_) {
      ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

}  // namespace dora2filter

PLUGINLIB_EXPORT_CLASS(dora2filter::Dora2Filter, filters::FilterBase<sensor_msgs::msg::LaserScan>)
