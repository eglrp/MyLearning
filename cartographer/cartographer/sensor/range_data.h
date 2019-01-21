/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {


/*
RangeData:
数据成员包括
1),原始位置,{x0,y0,z0}
2),返回点云,{x,y,z}
3),缺失点云,标识free space.
*/
// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {
  Eigen::Vector3f origin;//{x0,y0,z0},sensor坐标。
  PointCloud returns;//反射位置{x,y,z}，表征有物体反射。
  PointCloud misses;//无反射,自由空间
};

// Like 'RangeData', but with 'TimedPointClouds'.
struct TimedRangeData {
  Eigen::Vector3f origin;
  TimedPointCloud returns;
  TimedPointCloud misses;
};
//坐标变换
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);
//坐标变换
TimedRangeData TransformTimedRangeData(const TimedRangeData& range_data,
                                       const transform::Rigid3f& transform);
//根据min_z和max_z把不在z轴范围内的点云丢弃，剪裁到给定范围
// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);
//根据min_z和max_z把不在z轴范围内的点云丢弃，剪裁到给定范围
// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
TimedRangeData CropTimedRangeData(const TimedRangeData& range_data, float min_z,
                                  float max_z);

// Converts 'range_data' to a proto::RangeData.
proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
RangeData FromProto(const proto::RangeData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
