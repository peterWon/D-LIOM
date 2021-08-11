/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_RANGE_DATA_SYNCHRONIZER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_RANGE_DATA_SYNCHRONIZER_H_

#include <memory>
#include <deque>
#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {
//根据主雷达的时间戳Merge各个雷达的数据，返回一个按照采样时间戳升序排列的完整点云
//若输入的雷达是Velodyne的，则按照固定的扫描时间进行纠正
class RangeDataSynchronizer {
 public:
  explicit RangeDataSynchronizer(
      const std::vector<std::string>& expected_range_sensor_ids)
      : expected_sensor_ids_(expected_range_sensor_ids.begin(),
                             expected_range_sensor_ids.end()) {
    prior_sensor_id = expected_range_sensor_ids.front();
  }

  sensor::TimedPointCloudOriginData AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data,
      bool descrew);

 private:
  void StampRangeData(
    sensor::TimedPointCloudData& cloud,
    const double scanPeriod,   
    const int N_SCANS);
  void ToTimedPointCloudOriginData(
    const sensor::TimedPointCloudData& timed_point_cloud_data,
    sensor::TimedPointCloudOriginData& result);
  const std::set<std::string> expected_sensor_ids_;

  //主雷达只保持最新的一帧,辅雷达可以保留多帧,但一般最多两帧
  sensor::TimedPointCloudData prior_cloud_;
  std::deque<sensor::TimedPointCloudData> secondary_cloud_={};
  
  double current_start_ = -1.0;
  double current_end_ = -1.0;
  
  std::string prior_sensor_id = "";
};

}  // namespace mapping
}  // namespace cartographer
#endif //CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_SYNCHRONIZER_H_
