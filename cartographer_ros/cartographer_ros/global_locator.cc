#include "global_locator.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "msg_conversion.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"


void GlobalLocator::build_matchers_from_pbstream(const std::string& pbfilepath){
  ::cartographer::io::ProtoStreamReader reader(pbfilepath);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
//  auto options = load_options(cfg_file_dir,cfg_file_name);
  const auto& pose_graph = deserializer.pose_graph();

  ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(7);
  options.set_angular_search_window(30.*3.1415926 / 180.);
  options.set_branch_and_bound_depth(7);

  LOG(INFO) << "Loading submap from serialized data.";
  ::cartographer::mapping::proto::SerializedData proto;
  int i = 0;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (proto.has_submap()) {
      i++;
      const auto& submap_proto = proto.submap();
      const ::cartographer::mapping::SubmapId id{
          submap_proto.submap_id().trajectory_id(),
          submap_proto.submap_id().submap_index()};
      const ::cartographer::transform::Rigid3d global_submap_pose3d =
          ::cartographer::transform::ToRigid3(
              pose_graph.trajectory(id.trajectory_id)
                  .submap(id.submap_index)
                  .pose());
      const auto global_submap_pose2d =
              ::cartographer::transform::Project2D(global_submap_pose3d);

      ::cartographer::mapping::Submap2D submap(submap_proto.submap_2d());
      std::shared_ptr<::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D>
              scanmatcher=std::make_shared<::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D>(*(submap.grid()),options);

      std::shared_ptr<SubmapScanMatcher> submap_scan_matcher = std::make_shared<SubmapScanMatcher>(scanmatcher, global_submap_pose2d);
      _submap_scan_matchers.emplace_back(submap_scan_matcher);
    }
  }
  CHECK(reader.eof());
  LOG(INFO) << "Building pyramid done! Submap num is "<<i;
}

bool GlobalLocator::match(const sensor_msgs::LaserScan::ConstPtr &msg, GlobalPose2D &res){
  float score_tmp = 0.f, score = 0.f;
  constexpr float kMinScore = 0.1f;
  cartographer::transform::Rigid2d pose_estimate_tmp = cartographer::transform::Rigid2d::Identity();
  cartographer::transform::Rigid2d pose_estimate = cartographer::transform::Rigid2d::Identity();
  cartographer::sensor::PointCloud point_cloud = cartographer_ros::ToPointCloud((*msg));

  for(const auto& matcher: _submap_scan_matchers){
    if(!matcher->_scan_matcher_ptr->MatchFullSubmap(point_cloud, kMinScore, &score_tmp, &pose_estimate_tmp))
        continue;
    else{
      if(score_tmp > score){
        score = score_tmp;
        //pose_estimate = matcher->_submap_pose * pose_estimate_tmp;
        //really confused me! maybe the grid's maplimits has encoded the submap's global pose.
        pose_estimate = pose_estimate_tmp;
      }
    }
  }
  if(score>=_score_thresh){  
    res.x = pose_estimate.translation().coeff(0,0);;
    res.y = pose_estimate.translation().coeff(1,0);
    res.theta = pose_estimate.rotation().angle();
    return true;
  }
  return false;
}

bool GlobalLocator::match(const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg, GlobalPose2D &res){
  //todo,wz. align MultiEchoLaserScan with imu first then call scanmatcher's match function.
  LOG(ERROR)<<"Not implemented yet!";
  return false;
}

::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
GlobalLocator::load_options(const std::string cfg_file_dir,const std::string cfg_file_name){
  auto file_resolver = cartographer::common::make_unique<
      ::cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{cfg_file_dir});
  const std::string code =
      file_resolver->GetFileContentOrDie(cfg_file_name);
  ::cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  auto options = ::cartographer::mapping::scan_matching::
      CreateFastCorrelativeScanMatcherOptions2D(&lua_parameter_dictionary);
  return options;
}


