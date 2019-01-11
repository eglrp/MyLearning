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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {


/*
创建了一个节点对象node，
node中定义了MapBuilderBridge类对象map_builder_bridge_，map_builder_bridge_中定义了cartographer::mapping::MapBuilderInterface类的智能指针map_builder_，
该指针是ros程序和cartographer核算算法的接口

MapBuilderBridge中AddTrajectory调用接口类cartographer::mapping::MapBuilderInterface类的指针变量map_builder_的函数map_builder_->AddTrajectoryBuilder

MapBuilderBridge中LoadState调用接口类cartographer::mapping::MapBuilderInterface类的指针变量map_builder_的函数map_builder_->LoadState

MapBuilderBridge中的sensor_bridges_对象定义了接口类::cartographer::mapping::TrajectoryBuilderInterface* const指针变量trajectory_builder_，实现传感器数据的连接;
程序主题流程为node初始化相应主题后，相应的handle调用sensor_bridges_中的接口函数trajectory_builder_->AddSensorData。

*/

void Run() {//入口函数
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);//接收tf广播信息，缓冲10s
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);//读取配置信息，节点配置，轨迹配置

  auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
      node_options.map_builder_options);//定义cartographer::mapping::MapBuilder类的智能指针
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);//创建node，用于将ROS和SLAM进行连接
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);//加载状态信息，最终调用map_builder_->LoadState
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);//初始化相应的topic，开始跟踪
  }

  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
