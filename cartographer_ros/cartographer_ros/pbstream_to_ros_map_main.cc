/*
 * Copyright 2017 The Cartographer Authors
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

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "time_conversion.h"
#include <fstream>

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_filestem, "map", "Stem of the output files.");
DEFINE_double(resolution, 0.05, "Resolution of a grid cell in the drawn map.");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  std::ofstream ofs("nodes_info.txt");
  if(!ofs){
     LOG(ERROR)<<"Open node info output file failed!";
     return;
  }
  ofs<<"Timestamp in cartographer,\t Timestamp in ROS,\t Node's global 2d pose\n";


  const auto& pose_graph = deserializer.pose_graph();

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (proto.has_submap()) {
      const auto& submap = proto.submap();
      const ::cartographer::mapping::SubmapId id{
          submap.submap_id().trajectory_id(),
          submap.submap_id().submap_index()};
      const ::cartographer::transform::Rigid3d global_submap_pose =
          ::cartographer::transform::ToRigid3(
              pose_graph.trajectory(id.trajectory_id)
                  .submap(id.submap_index)
                  .pose());

//      const auto trans = pose_graph.trajectory(id.trajectory_id)
//          .submap(id.submap_index)
//          .pose();
//      double x = trans.rotation().x();
//      double y = trans.rotation().y();
//      double z = trans.rotation().z();
//      double w = trans.rotation().w();
//      LOG(INFO)<<"rotation quaterniond "<<x<<","<<y<<","<<z<<","<<w;
//      const auto submap_global_pose_2d =
//              ::cartographer::transform::Project2D(global_submap_pose);
//      const auto& x = submap_global_pose_2d.translation().coeff(0,0);;
//      const auto& y = submap_global_pose_2d.translation().coeff(1,0);
//      double theta = submap_global_pose_2d.rotation().angle();
//      LOG(INFO)<<"The submap pose 2d: ("<<x<<","<<y<<","<<theta<<")";

      FillSubmapSlice(global_submap_pose, submap, &submap_slices[id]);
    }
    if(proto.has_node()){//add by wz,for test.
      const auto& node = proto.node();

      const auto& trajectory_id = node.node_id().trajectory_id();
      const auto& node_index = node.node_id().node_index();

      const auto node_global_pose = ::cartographer::transform::ToRigid3(
            pose_graph.trajectory(trajectory_id)
                .node(node_index)
                .pose());
      const auto node_global_pose_2d =
              ::cartographer::transform::Project2D(node_global_pose);
      const auto& timestamp = pose_graph.trajectory(trajectory_id)
                              .node(node_index)
                              .timestamp();
      const auto& x = node_global_pose_2d.translation().coeff(0,0);;
      const auto& y = node_global_pose_2d.translation().coeff(1,0);
      double theta = node_global_pose_2d.rotation().angle();
      const auto& time_in_ros = ToRos(::cartographer::common::FromUniversal(timestamp));
      LOG(INFO)<<"The node's time stamp in ros:"<<time_in_ros<<", pose: ("<<x<<","<<y<<","<<theta<<").";

      ofs<<timestamp<<","<<time_in_ros<<","<<x<<","<<y<<","<<theta<<"\n";
    }
  }
  CHECK(reader.eof());
  ofs.close();

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_map_filestem.empty()) << "-map_filestem is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_filestem,
                          FLAGS_resolution);
}
