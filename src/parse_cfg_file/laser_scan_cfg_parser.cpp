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

#include "laser_scan_cfg_parser.h"
#include "rapidjson/filereadstream.h"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace livox_ros {

bool LaserScanConfigParser::ParseLaserScanConfig(const rapidjson::Document &doc,
                                                 LaserScanConfig &scan_config) {
  if (!doc.HasMember("laser_scan_config")) {
    // Use default configuration if not specified
    scan_config = LaserScanConfig();
    return true;
  }

  const rapidjson::Value &config_value = doc["laser_scan_config"];
  return ParseLaserScanFromValue(config_value, scan_config);
}

bool LaserScanConfigParser::ParseLaserScanConfig(
    const std::string &config_file_path, LaserScanConfig &scan_config) {
  FILE *raw_file = std::fopen(config_file_path.c_str(), "rb");
  if (!raw_file) {
    std::cout << "Failed to open config file: " << config_file_path
              << std::endl;
    return false;
  }

  constexpr uint32_t kMaxBufferSize = 0x8000; // 32k bytes
  char read_buffer[kMaxBufferSize];
  rapidjson::FileReadStream config_file(raw_file, read_buffer,
                                        sizeof(read_buffer));
  rapidjson::Document doc;

  bool result = false;
  do {
    if (doc.ParseStream(config_file).HasParseError()) {
      std::cout << "Failed to parse config JSON" << std::endl;
      break;
    }

    result = ParseLaserScanConfig(doc, scan_config);
  } while (false);

  std::fclose(raw_file);
  return result;
}

bool LaserScanConfigParser::ParseLaserScanFromValue(
    const rapidjson::Value &value, LaserScanConfig &scan_config) {
  // Initialize with defaults
  scan_config = LaserScanConfig();

  if (!value.IsObject()) {
    std::cout << "laser_scan_config must be an object" << std::endl;
    return false;
  }

  // Parse enable_scan
  if (value.HasMember("enable_scan")) {
    if (value["enable_scan"].IsBool()) {
      scan_config.enable_scan = value["enable_scan"].GetBool();
    } else {
      std::cout << "enable_scan must be a boolean" << std::endl;
      return false;
    }
  }

  // Parse min_height
  if (value.HasMember("min_height")) {
    if (value["min_height"].IsNumber()) {
      scan_config.min_height = value["min_height"].GetDouble();
    } else {
      std::cout << "min_height must be a number" << std::endl;
      return false;
    }
  }

  // Parse max_height
  if (value.HasMember("max_height")) {
    if (value["max_height"].IsNumber()) {
      scan_config.max_height = value["max_height"].GetDouble();
    } else {
      std::cout << "max_height must be a number" << std::endl;
      return false;
    }
  }

  // Parse angle_min
  if (value.HasMember("angle_min")) {
    if (value["angle_min"].IsNumber()) {
      scan_config.angle_min = value["angle_min"].GetDouble();
    } else {
      std::cout << "angle_min must be a number" << std::endl;
      return false;
    }
  }

  // Parse angle_max
  if (value.HasMember("angle_max")) {
    if (value["angle_max"].IsNumber()) {
      scan_config.angle_max = value["angle_max"].GetDouble();
    } else {
      std::cout << "angle_max must be a number" << std::endl;
      return false;
    }
  }

  // Parse angle_increment
  if (value.HasMember("angle_increment")) {
    if (value["angle_increment"].IsNumber()) {
      scan_config.angle_increment = value["angle_increment"].GetDouble();
    } else {
      std::cout << "angle_increment must be a number" << std::endl;
      return false;
    }
  }

  // Parse scan_time
  if (value.HasMember("scan_time")) {
    if (value["scan_time"].IsNumber()) {
      scan_config.scan_time = value["scan_time"].GetDouble();
    } else {
      std::cout << "scan_time must be a number" << std::endl;
      return false;
    }
  }

  // Parse range_min
  if (value.HasMember("range_min")) {
    if (value["range_min"].IsNumber()) {
      scan_config.range_min = value["range_min"].GetDouble();
    } else {
      std::cout << "range_min must be a number" << std::endl;
      return false;
    }
  }

  // Parse range_max
  if (value.HasMember("range_max")) {
    if (value["range_max"].IsNumber()) {
      scan_config.range_max = value["range_max"].GetDouble();
    } else {
      std::cout << "range_max must be a number" << std::endl;
      return false;
    }
  }

  // Parse use_inf
  if (value.HasMember("use_inf")) {
    if (value["use_inf"].IsBool()) {
      scan_config.use_inf = value["use_inf"].GetBool();
    } else {
      std::cout << "use_inf must be a boolean" << std::endl;
      return false;
    }
  }

  // Parse inf_epsilon
  if (value.HasMember("inf_epsilon")) {
    if (value["inf_epsilon"].IsNumber()) {
      scan_config.inf_epsilon = value["inf_epsilon"].GetDouble();
    } else {
      std::cout << "inf_epsilon must be a number" << std::endl;
      return false;
    }
  }

  // Parse target_frame
  if (value.HasMember("target_frame")) {
    if (value["target_frame"].IsString()) {
      scan_config.target_frame = value["target_frame"].GetString();
    } else {
      std::cout << "target_frame must be a string" << std::endl;
      return false;
    }
  }

  // Parse transform_tolerance
  if (value.HasMember("transform_tolerance")) {
    if (value["transform_tolerance"].IsNumber()) {
      scan_config.transform_tolerance =
          value["transform_tolerance"].GetDouble();
    } else {
      std::cout << "transform_tolerance must be a number" << std::endl;
      return false;
    }
  }

  return true;
}

} // namespace livox_ros
