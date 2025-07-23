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

#include "pointcloud_to_laserscan_converter.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace livox_ros {

PointCloudToLaserScanConverter::PointCloudToLaserScanConverter() {
  // Initialize with default config
  config_ = LaserScanConfig();
}

void PointCloudToLaserScanConverter::SetConfig(const LaserScanConfig &config) {
  config_ = config;
}

#ifdef BUILDING_ROS1
void PointCloudToLaserScanConverter::ConvertToLaserScan(
    const std::vector<PointXyzlt> &points, const uint64_t timestamp,
    const std::string &frame_id, sensor_msgs::LaserScan &scan_msg) {
#elif defined BUILDING_ROS2
void PointCloudToLaserScanConverter::ConvertToLaserScan(
    const std::vector<PointXyzlt> &points, const uint64_t timestamp,
    const std::string &frame_id, sensor_msgs::msg::LaserScan &scan_msg) {
#endif

  if (!config_.enable_scan) {
    return;
  }

  // Initialize the laser scan message
  InitializeLaserScanMessage(timestamp, frame_id, scan_msg);

  // Calculate the number of ranges
  uint32_t ranges_size = std::ceil((scan_msg.angle_max - scan_msg.angle_min) /
                                   scan_msg.angle_increment);

  // Initialize ranges with infinity or max_range + epsilon
  if (config_.use_inf) {
    scan_msg.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
  } else {
    scan_msg.ranges.assign(ranges_size,
                           scan_msg.range_max + config_.inf_epsilon);
  }

  // Process each point in the point cloud
  for (const auto &point : points) {
    // Skip invalid points
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      continue;
    }

    // Check if point is within height range
    if (point.z > config_.max_height || point.z < config_.min_height) {
      continue;
    }

    // Calculate range
    double range = CalculateRange(point);

    // Check if range is within limits
    if (range < config_.range_min || range > config_.range_max) {
      continue;
    }

    // Calculate angle
    double angle = CalculateAngle(point);

    // Check if angle is within scan range
    if (angle < scan_msg.angle_min || angle > scan_msg.angle_max) {
      continue;
    }

    // Calculate the index in the ranges array
    int index = CalculateIndex(angle);

    // Update the range if this point is closer
    if (index >= 0 && index < static_cast<int>(ranges_size)) {
      if (range < scan_msg.ranges[index]) {
        scan_msg.ranges[index] = range;
      }
    }
  }
}

bool PointCloudToLaserScanConverter::IsPointInRange(
    const PointXyzlt &point) const {
  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
    return false;
  }

  if (point.z > config_.max_height || point.z < config_.min_height) {
    return false;
  }

  double range = CalculateRange(point);
  if (range < config_.range_min || range > config_.range_max) {
    return false;
  }

  double angle = CalculateAngle(point);
  if (angle < config_.angle_min || angle > config_.angle_max) {
    return false;
  }

  return true;
}

double
PointCloudToLaserScanConverter::CalculateRange(const PointXyzlt &point) const {
  return std::sqrt(point.x * point.x + point.y * point.y);
}

double
PointCloudToLaserScanConverter::CalculateAngle(const PointXyzlt &point) const {
  return std::atan2(point.y, point.x);
}

int PointCloudToLaserScanConverter::CalculateIndex(double angle) const {
  return static_cast<int>((angle - config_.angle_min) /
                          config_.angle_increment);
}

#ifdef BUILDING_ROS1
void PointCloudToLaserScanConverter::InitializeLaserScanMessage(
    const uint64_t timestamp, const std::string &frame_id,
    sensor_msgs::LaserScan &scan_msg) const {
#elif defined BUILDING_ROS2
void PointCloudToLaserScanConverter::InitializeLaserScanMessage(
    const uint64_t timestamp, const std::string &frame_id,
    sensor_msgs::msg::LaserScan &scan_msg) const {
#endif

  // Set frame information
  scan_msg.header.frame_id =
      config_.target_frame.empty() ? frame_id : config_.target_frame;

#ifdef BUILDING_ROS1
  static uint32_t seq = 0;
  scan_msg.header.seq = seq++;
  scan_msg.header.stamp = ros::Time(timestamp / 1000000000.0);
#elif defined BUILDING_ROS2
  scan_msg.header.stamp = rclcpp::Time(timestamp);
#endif

  // Set scan parameters
  scan_msg.angle_min = config_.angle_min;
  scan_msg.angle_max = config_.angle_max;
  scan_msg.angle_increment = config_.angle_increment;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = config_.scan_time;
  scan_msg.range_min = config_.range_min;
  scan_msg.range_max = config_.range_max;
}

} // namespace livox_ros
