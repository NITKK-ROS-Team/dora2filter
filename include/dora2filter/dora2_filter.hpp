#ifndef DORA2FILTER__DORA2_FILTER_HPP_
#define DORA2FILTER__DORA2_FILTER_HPP_

#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace dora2filter
{

class Dora2Filter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
public:
  Dora2Filter() = default;
  ~Dora2Filter() override = default;

  using filters::FilterBase<sensor_msgs::msg::LaserScan>::configure;

  bool configure() override;

  bool update(
    const sensor_msgs::msg::LaserScan & input_scan,
    sensor_msgs::msg::LaserScan & output_scan) override;

private:
  double threshold_{0.83};
};

}  // namespace dora2filter

#endif  // DORA2FILTER__DORA2_FILTER_HPP_
