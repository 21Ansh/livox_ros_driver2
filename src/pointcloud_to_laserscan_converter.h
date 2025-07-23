//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef POINTCLOUD_TO_LASERSCAN_CONVERTER_H_
#define POINTCLOUD_TO_LASERSCAN_CONVERTER_H_

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "comm/comm.h"
#include "include/ros_headers.h"

namespace livox_ros {

struct LaserScanConfig {
  bool enable_scan = false;
  double min_height = -1.0;
  double max_height = 1.0;
  double angle_min = -M_PI;
  double angle_max = M_PI;
  double angle_increment = M_PI / 180.0; // 1 degree
  double scan_time = 1.0 / 30.0;         // 30 Hz
  double range_min = 0.45;
  double range_max = 100.0;
  bool use_inf = true;
  double inf_epsilon = 1.0;
  std::string target_frame = "";
  double transform_tolerance = 0.01;
};

class PointCloudToLaserScanConverter {
public:
  PointCloudToLaserScanConverter();
  ~PointCloudToLaserScanConverter() = default;

  void SetConfig(const LaserScanConfig &config);
  LaserScanConfig GetConfig() const { return config_; }

#ifdef BUILDING_ROS1
  void ConvertToLaserScan(const std::vector<PointXyzlt> &points,
                          const uint64_t timestamp, const std::string &frame_id,
                          sensor_msgs::LaserScan &scan_msg);
#elif defined BUILDING_ROS2
  void ConvertToLaserScan(const std::vector<PointXyzlt> &points,
                          const uint64_t timestamp, const std::string &frame_id,
                          sensor_msgs::msg::LaserScan &scan_msg);
#endif

  bool IsEnabled() const { return config_.enable_scan; }

private:
  LaserScanConfig config_;

  // Helper methods
  bool IsPointInRange(const PointXyzlt &point) const;
  double CalculateRange(const PointXyzlt &point) const;
  double CalculateAngle(const PointXyzlt &point) const;
  int CalculateIndex(double angle) const;
  void InitializeLaserScanMessage(const uint64_t timestamp,
                                  const std::string &frame_id,
#ifdef BUILDING_ROS1
                                  sensor_msgs::LaserScan &scan_msg) const;
#elif defined BUILDING_ROS2
                                  sensor_msgs::msg::LaserScan &scan_msg) const;
#endif
};

} // namespace livox_ros

#endif // POINTCLOUD_TO_LASERSCAN_CONVERTER_H_
