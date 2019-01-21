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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/mapping/proto/3d/submaps_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//real_time_correlative_scan_matcher_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// TODO(spielawa): Adjust metrics for multi-trajectory. So far we assume a
// single trajectory.
static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamVoxelFilterFraction = metrics::Gauge::Null();
static auto* kLocalSlamScanMatcherFraction = metrics::Gauge::Null();
static auto* kLocalSlamInsertIntoSubmapFraction = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

LocalTrajectoryBuilder3D::LocalTrajectoryBuilder3D(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          absl::make_unique<scan_matching::RealTimeCorrelativeScanMatcher3D>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(absl::make_unique<scan_matching::CeresScanMatcher3D>(
          options_.ceres_scan_matcher_options())),
      accumulated_range_data_{Eigen::Vector3f::Zero(), {}, {}},
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder3D::~LocalTrajectoryBuilder3D() {}

std::unique_ptr<transform::Rigid3d> LocalTrajectoryBuilder3D::ScanMatch(
    const transform::Rigid3d& pose_prediction,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking) {
  if (active_submaps_.submaps().empty()) {//如果submap为空直接返回pose_prediction
    return absl::make_unique<transform::Rigid3d>(pose_prediction);
  }
  std::shared_ptr<const mapping::Submap3D> matching_submap =
      active_submaps_.submaps().front();//取得用于匹配的submap
  transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;//初始化位姿优化的初始值
  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'initial_ceres_pose' as an output argument.
    const transform::Rigid3d initial_pose = initial_ceres_pose;
    const double score = real_time_correlative_scan_matcher_->Match(//调用匹配函数,得到匹配结果initial_ceres_pose,基于voxol进行匹配
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(//调用ceres优化位姿
      (matching_submap->local_pose().inverse() * pose_prediction).translation(),//目标位移
      initial_ceres_pose,//位姿,match用其中的rotation
      {{&high_resolution_point_cloud_in_tracking,
        &matching_submap->high_resolution_hybrid_grid()},//点云和栅格地图
       {&low_resolution_point_cloud_in_tracking,
        &matching_submap->low_resolution_hybrid_grid()}},
      &pose_observation_in_submap, &summary);//优化结果和summary
  kCeresScanMatcherCostMetric->Observe(summary.final_cost);
  const double residual_distance = (pose_observation_in_submap.translation() -
                                    initial_ceres_pose.translation())
                                       .norm();//计算优化后位移的变化
  kScanMatcherResidualDistanceMetric->Observe(residual_distance);
  const double residual_angle =
      pose_observation_in_submap.rotation().angularDistance(
          initial_ceres_pose.rotation());//计算优化后姿态的旋转
  kScanMatcherResidualAngleMetric->Observe(residual_angle);
  return absl::make_unique<transform::Rigid3d>(matching_submap->local_pose() *
                                               pose_observation_in_submap);
}

void LocalTrajectoryBuilder3D::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {//std::unique_ptr<mapping::PoseExtrapolator> extrapolator_;
    extrapolator_->AddImuData(imu_data);
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
}


/*
 *输入
 */
std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  const auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);//将非同步的数据添加到同步器内,返回sensor::TimedPointCloudOriginData
  if (synchronized_data.ranges.empty()) {//同步数据ranges为空，返回空指针
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  const common::Time& current_sensor_time = synchronized_data.time;
  if (extrapolator_ == nullptr) {//等待有IMU数据以后才运行，否则返回
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
  const common::Time time_first_point =
      current_sensor_time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);//获取第一个点的时间
  if (time_first_point < extrapolator_->GetLastPoseTime()) {//判断Extrapolator是否完成初始化，否则返回空指针
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }
/*  struct RangeMeasurement {
    TimedRangefinderPoint point_time;
    size_t origin_index;
  };
*/
  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> hits =
      sensor::VoxelFilter(0.5f * options_.voxel_filter_size())
          .Filter(synchronized_data.ranges);//对雷达测量数据进行体素滤波

  std::vector<transform::Rigid3f> hits_poses;
  hits_poses.reserve(hits.size());//将容量设为hits长度
  bool warned = false;

  for (const auto& hit : hits) {//遍历hits中的所有测量数据
    common::Time time_point =//获取当前点的时间
        current_sensor_time + common::FromSeconds(hit.point_time.time);
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {//如果当前点时间比上一次处理的时间小，告警
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();//设置当前点的时间为上一次处理的时间
    }
    hits_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is not used.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  for (size_t i = 0; i < hits.size(); ++i) {
    sensor::RangefinderPoint hit_in_local =
        hits_poses[i] * sensor::ToRangefinderPoint(hits[i].point_time);//hits转换为RangefinderPoint，并用转移矩阵进行坐标变换
    const Eigen::Vector3f origin_in_local =
        hits_poses[i] * synchronized_data.origins.at(hits[i].origin_index);//求hits对应的源坐标
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;//计算偏移矢量
    const float range = delta.norm();//求模长
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // We insert a ray cropped to 'max_range' as a miss for hits beyond the
        // maximum range. This way the free space up to the maximum range will
        // be updated.
        hit_in_local.position =
            origin_in_local + options_.max_range() / range * delta;//如果长度超出了设置的最大长度，将长度缩放至最大长度
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.num_accumulated_range_data()) {//accumulated_range_data_计数达到设定的大小
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();//计算sensor数据间隔时间
    }
    last_sensor_time_ = current_sensor_time;//更新记录的last_sensor_time_
    num_accumulated_ = 0;//accumulated_range_data_计数复位

    transform::Rigid3f current_pose =
        extrapolator_->ExtrapolatePose(current_sensor_time).cast<float>();//更新位姿

    const auto voxel_filter_start = std::chrono::steady_clock::now();//获取当前时间记为voxel_filter_start
    const sensor::RangeData filtered_range_data = {//对accumulated_range_data_的returns和misses进行滤波，得到滤波后的filtered_range_data
        current_pose.translation(),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.returns),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.misses)};
    const auto voxel_filter_stop = std::chrono::steady_clock::now();//获取当前时间
    const auto voxel_filter_duration = voxel_filter_stop - voxel_filter_start;//计算滤波消耗的时间

    if (sensor_duration.has_value()) {
      const double voxel_filter_fraction =
          common::ToSeconds(voxel_filter_duration) /
          common::ToSeconds(sensor_duration.value());//计算滤波分数=滤波时间长度/传感器时间间隔
      kLocalSlamVoxelFilterFraction->Set(voxel_filter_fraction);//存储滤波分数
    }
    //上述代码利用实时的pose将三维点在local地图坐标下拼接，消除激光雷达运动畸变

    return AddAccumulatedRangeData(
        current_sensor_time,
        sensor::TransformRangeData(filtered_range_data, current_pose.inverse()),//利用current_pose将local坐标下的表示的扫描点反向投影到sensor坐标下
        sensor_duration);
  }
  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_tracking,//输入滤波处理过的点云数据
    const absl::optional<common::Duration>& sensor_duration) {
  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }
  const transform::Rigid3d pose_prediction =
      extrapolator_->ExtrapolatePose(time);//更新位姿信息

  const auto scan_matcher_start = std::chrono::steady_clock::now();//记录match操作开始时间

  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());//创建一个自适应滤波器(高分辨率)
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);//对returns点云进行滤波
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(//创建一个自适应滤波器(低分辨率)
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);//对returns点云进行滤波
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }

  std::unique_ptr<transform::Rigid3d> pose_estimate =
      ScanMatch(pose_prediction, low_resolution_point_cloud_in_tracking,
                high_resolution_point_cloud_in_tracking);//调用匹配函数,返回优化后的位姿
  if (pose_estimate == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  extrapolator_->AddPose(time, *pose_estimate);

  const auto scan_matcher_stop = std::chrono::steady_clock::now();
  const auto scan_matcher_duration = scan_matcher_stop - scan_matcher_start;//计算match用时
  if (sensor_duration.has_value()) {
    const double scan_matcher_fraction =
        common::ToSeconds(scan_matcher_duration) /
        common::ToSeconds(sensor_duration.value());
    kLocalSlamScanMatcherFraction->Set(scan_matcher_fraction);//记录match用时比例
  }

  const Eigen::Quaterniond gravity_alignment =
      extrapolator_->EstimateGravityOrientation(time);
  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, pose_estimate->cast<float>());//使用pose_estimate对点云进行变换得到local坐标系下的点云

  const auto insert_into_submap_start = std::chrono::steady_clock::now();
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, *pose_estimate,
      gravity_alignment);//点云插入到submap中
  const auto insert_into_submap_stop = std::chrono::steady_clock::now();

  const auto insert_into_submap_duration =
      insert_into_submap_stop - insert_into_submap_start;//计算点云插入的时间
  if (sensor_duration.has_value()) {
    const double insert_into_submap_fraction =
        common::ToSeconds(insert_into_submap_duration) /
        common::ToSeconds(sensor_duration.value());
    kLocalSlamInsertIntoSubmapFraction->Set(insert_into_submap_fraction);
  }
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));//计算kLocalSlamRealTimeRatio
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);//计算kLocalSlamCpuRealTimeRatio
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  return absl::make_unique<MatchingResult>(MatchingResult{
      time, *pose_estimate, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

void LocalTrajectoryBuilder3D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

std::unique_ptr<LocalTrajectoryBuilder3D::InsertionResult>
LocalTrajectoryBuilder3D::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  const Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());

  const Eigen::Quaterniond local_from_gravity_aligned =
      pose_estimate.rotation() * gravity_alignment.inverse();//计算local坐标系与水平坐标系的旋转
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      active_submaps_.InsertData(filtered_range_data_in_local,
                                 local_from_gravity_aligned,
                                 rotational_scan_matcher_histogram_in_gravity);//点云插入到激活的submap中
  return absl::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram_in_gravity,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder3D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});

  auto* voxel_filter_fraction = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_voxel_filter_fraction",
      "Fraction of total sensor time taken up by voxel filter.");
  kLocalSlamVoxelFilterFraction = voxel_filter_fraction->Add({});

  auto* scan_matcher_fraction = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_scan_matcher_fraction",
      "Fraction of total sensor time taken up by scan matcher.");
  kLocalSlamScanMatcherFraction = scan_matcher_fraction->Add({});

  auto* insert_into_submap_fraction = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_insert_into_submap_fraction",
      "Fraction of total sensor time taken up by inserting into submap.");
  kLocalSlamInsertIntoSubmapFraction = insert_into_submap_fraction->Add({});

  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_3d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});

  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_3d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_3d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_3d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
