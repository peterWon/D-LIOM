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

#include "cartographer/mapping/internal/3d/range_data_synchronizer.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

sensor::TimedPointCloudOriginData RangeDataSynchronizer::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data,
    bool descrew) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(wz): not efficient, to improve
  sensor::TimedPointCloudOriginData result = {};
  sensor::TimedPointCloudData point_cloud = timed_point_cloud_data;
  if(descrew){
    StampRangeData(point_cloud, 0.1, 64);
  }
  if(sensor_id == prior_sensor_id){
    if(secondary_cloud_.empty()){
      ToTimedPointCloudOriginData(point_cloud, result);
      return result;
    }else{
      current_end_ = common::ToSecondsStamp(point_cloud.time);
      current_start_ = current_end_ + point_cloud.ranges.front()[3];
      // pop old clouds
      while(1){
        if(secondary_cloud_.empty()){
          ToTimedPointCloudOriginData(point_cloud, result);
          return result;
        }
        if(common::ToSecondsStamp(
          secondary_cloud_.front().time) < current_start_){
          secondary_cloud_.pop_front();
        }else{
          break;
        }
      }
      auto secondary_lidar_time = common::ToSecondsStamp(
        secondary_cloud_.front().time);
      // secondary lidar too fast...should check it
      if(secondary_lidar_time + secondary_cloud_.front().ranges.front()[3] 
          > current_end_){
        LOG(WARNING) << "The secondary lidar may be too fast, check it...";
        ToTimedPointCloudOriginData(point_cloud, result);
        return result;
      }
      // find overlap and merge data.
      auto ranges = secondary_cloud_.front().ranges;
      int i_start = -1;
      int i_end = -1;
      for(int i = 0; i < ranges.size(); ++i){
        if(secondary_lidar_time + ranges[i][3] >= current_start_  
            && secondary_lidar_time + ranges[i][3] <= current_end_
          && i_start == -1){
          i_start = i;
        }
        if(i_start != -1 && secondary_lidar_time + ranges[i][3] > current_end_){
          i_end = i - 1;
          break;
        }
      }
      CHECK(i_start != -1)<<"Something wrong, check it...";
      if(i_end == -1) i_end = ranges.size() - 1;
      
      result.ranges.resize(point_cloud.ranges.size() + i_end - i_start + 1);

      sensor::TimedPointCloudOriginData::RangeMeasurement range;      
      result.time = point_cloud.time;
      result.origins.push_back(point_cloud.origin);
      range.origin_index = 0;
      for(size_t i = 0; i < point_cloud.ranges.size(); ++i){
        range.point_time = timed_point_cloud_data.ranges[i];
        result.ranges[i] = range;
      }
      range.origin_index = 1;
      result.origins.push_back(secondary_cloud_.front().origin);
      double relative_t = 0.;
      for(size_t i = i_start; i <= i_end; ++i){
        range.point_time = secondary_cloud_.front().ranges[i];
        relative_t = secondary_cloud_.front().ranges[i][3];
        range.point_time[3] = relative_t + secondary_lidar_time - current_end_;
        result.ranges[point_cloud.ranges.size() + i - i_start] = range;
      }
      std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time[3] < b.point_time[3];
            });
      return result;
    }
  }else{
    secondary_cloud_.push_back(point_cloud);
  }
  return result;
}

void RangeDataSynchronizer::StampRangeData(
    sensor::TimedPointCloudData& cloud,
    const double scanPeriod,   
    const int N_SCANS){
  int cloudSize = cloud.ranges.size();
  if(cloudSize < 2) return;
  // very very naive implementation.
  const double duration =  scanPeriod / (cloudSize - 1);
  for (int i = 0; i < cloudSize; i++){
    cloud.ranges[i][3] = -scanPeriod + i * duration;
  }
  cloud.ranges.back()[3] = 0.;
  /* float startOri = -atan2(
    cloud.ranges[0][1], 
    cloud.ranges[0][0]);
  float endOri = -atan2(
    cloud.ranges[cloudSize-1][1], 
    cloud.ranges[cloudSize-1][0]) + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI){
    endOri -= 2 * M_PI;
  }else if (endOri - startOri < M_PI){
    endOri += 2 * M_PI;
  }
  
  bool halfPassed = false;
  int count = cloudSize;
  const float angle_ratio = 180. / M_PI;
  
  float x,y,z;
  for (int i = 0; i < cloudSize; i++){
    x = cloud.ranges[i][0];
    y = cloud.ranges[i][1];
    z = cloud.ranges[i][2];

    float angle = atan(z / sqrt(x * x + y * y)) * angle_ratio;
   
    float ori = -atan2(y, x);
    if (!halfPassed){ 
      if (ori < startOri - M_PI / 2){
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }
      if (ori - startOri > M_PI){
        halfPassed = true;
      }
    }else{
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2){
        ori += 2 * M_PI;
      }else if (ori > endOri + M_PI / 2){
        ori -= 2 * M_PI;
      }
    }
    //the last point's point_time[3] is 0.
    float ratio = (ori - startOri) / (endOri - startOri);
    cloud.ranges[i][3] = -scanPeriod + scanPeriod * ratio;
  } */
}

void RangeDataSynchronizer::ToTimedPointCloudOriginData(
    const sensor::TimedPointCloudData& timed_point_cloud_data,
    sensor::TimedPointCloudOriginData& result){
  result.time = timed_point_cloud_data.time;
  result.ranges.resize(timed_point_cloud_data.ranges.size());
  result.origins = {};
  result.origins.push_back(timed_point_cloud_data.origin);

  sensor::TimedPointCloudOriginData::RangeMeasurement range;
  for(size_t i = 0; i < timed_point_cloud_data.ranges.size(); ++i){
    range.point_time = timed_point_cloud_data.ranges[i];
    range.origin_index = 0;
    result.ranges[i] = range;
  }
}

}  // namespace mapping
}  // namespace cartographer
