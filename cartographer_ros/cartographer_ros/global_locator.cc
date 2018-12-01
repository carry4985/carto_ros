#include "global_locator.h"
#include <cmath>
#include <map>

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

#include "glog/logging.h"
#include "msg_conversion.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"


GlobalLocator::GlobalLocator(const std::string &pbfilepath){
    loadSubmaps(pbfilepath,_submap_scan_matchers_higher);
    _pb_filename = pbfilepath;
    _inited = true;
    _two_stage_mode = false;
}

GlobalLocator::GlobalLocator(const std::string &high_resolution_pbfilepath,
                             const std::string &low_resolution_pbfilepath){
    _pb_filename = high_resolution_pbfilepath;
    loadSubmaps(high_resolution_pbfilepath,_submap_scan_matchers_higher);
    loadSubmaps(low_resolution_pbfilepath,_submap_scan_matchers_lower);
    _inited = true;
    _two_stage_mode = true;
}

void GlobalLocator::loadSubmaps(const std::string& pbfilepath,
                                 std::vector<std::shared_ptr<SubmapScanMatcher>>& matcher_ptr){
  ::cartographer::io::ProtoStreamReader reader(pbfilepath);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
//  auto options = loadOptions(cfg_file_dir,cfg_file_name);
  const auto& pose_graph = deserializer.pose_graph();

  ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D options;
  options.set_linear_search_window(7);
  options.set_angular_search_window(30.*3.1415926 / 180.);
  options.set_branch_and_bound_depth(7);//important, this para decides the runing time.

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

      auto max_coordinate = submap.grid()->limits().max();
      const ::cartographer::transform::Rigid2d center =
              ::cartographer::transform::Rigid2d::Translation(::cartographer::transform::Rigid2d::Vector((max_coordinate.x()+global_submap_pose2d.translation().coeff(0,0)) / 2,
                                                 (max_coordinate.y()+global_submap_pose2d.translation().coeff(1,0)) / 2));
      std::shared_ptr<::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D>
              scanmatcher=std::make_shared<::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D>(*(submap.grid()),options);

      std::shared_ptr<SubmapScanMatcher> submap_scan_matcher = std::make_shared<SubmapScanMatcher>(scanmatcher, global_submap_pose2d, center);
      matcher_ptr.emplace_back(submap_scan_matcher);
    }
  }
  CHECK(reader.eof());
  LOG(INFO) << "Building pyramid done! Submap num is "<<i;
}

void GlobalLocator::loadHistMap(const std::string &hist_file_path){
   _hist_matcher.loadHistgramMapData(hist_file_path);
}

bool GlobalLocator::matchWithHistmap(const sensor_msgs::LaserScan::ConstPtr &msg, GlobalPose2D &res){
   std::vector<cv::Point2f> pts = _hist_matcher.getCandidates(msg, 20);
   float score_tmp = 0.f, score = 0.f;
   constexpr float kMinScore = 0.1f;
   cartographer::transform::Rigid2d pose_estimate_tmp = cartographer::transform::Rigid2d::Identity();
   cartographer::transform::Rigid2d pose_estimate = cartographer::transform::Rigid2d::Identity();
   cartographer::sensor::PointCloud point_cloud = cartographer_ros::ToPointCloud((*msg));

   if(_submap_scan_matchers_higher.empty())return false;
   std::vector<int> matched_ids;
   for(const auto& pt: pts){
     const ::cartographer::transform::Rigid2d carto_pt =
             ::cartographer::transform::Rigid2d::Translation(::cartographer::transform::Rigid2d::Vector(pt.x, pt.y));
     int index = getNearestSubmap(carto_pt);
     if(std::find(matched_ids.begin(),matched_ids.end(),index)!=matched_ids.end()){
       continue;
     }
     matched_ids.push_back(index);
     if(!_submap_scan_matchers_higher[index]->_scan_matcher_ptr->MatchFullSubmap(point_cloud, kMinScore,
                                                     &score_tmp, &pose_estimate_tmp))
         continue;
     else{
//       LOG(INFO)<<score_tmp;
       if(score_tmp > score){
         score = score_tmp;
         pose_estimate = pose_estimate_tmp;
       }
     }

     if(score>=_score_thresh_higher) break;
   }

//   writeSubmaps(matched_ids);

   //whatever, return the final matched pose.
   res.x = pose_estimate.translation().coeff(0,0);;
   res.y = pose_estimate.translation().coeff(1,0);
   res.theta = pose_estimate.rotation().angle();
   return true;
}

//todo:find out why the matched result at timestamp of 700s worse than realtime mapping.
bool GlobalLocator::match(const sensor_msgs::LaserScan::ConstPtr &msg, GlobalPose2D &res){
  float score_tmp = 0.f, score = 0.f;
  constexpr float kMinScore = 0.1f;
  cartographer::transform::Rigid2d pose_estimate_tmp = cartographer::transform::Rigid2d::Identity();
  cartographer::transform::Rigid2d pose_estimate = cartographer::transform::Rigid2d::Identity();
  cartographer::sensor::PointCloud point_cloud = cartographer_ros::ToPointCloud((*msg));

  if(!_two_stage_mode){
      for(const auto& matcher: _submap_scan_matchers_higher){
        if(!matcher->_scan_matcher_ptr->MatchFullSubmap(point_cloud, kMinScore,
                                                        &score_tmp, &pose_estimate_tmp))
            continue;
        else{
          if(score_tmp > score){
            score = score_tmp;
            //pose_estimate = matcher->_origin * pose_estimate_tmp;
            //really confused me! maybe the grid's maplimits has encoded the submap's global pose.
            pose_estimate = pose_estimate_tmp;
          }
        }
      }
      if(score>=_score_thresh_higher){
        res.x = pose_estimate.translation().coeff(0,0);;
        res.y = pose_estimate.translation().coeff(1,0);
        res.theta = pose_estimate.rotation().angle();
        return true;
      }
  } else {
      struct timeval tpstart,tpend;
      float timeuse;
      gettimeofday(&tpstart,NULL);

      for(const auto& matcher: _submap_scan_matchers_lower){
        if(!matcher->_scan_matcher_ptr->MatchFullSubmap(point_cloud, kMinScore,
                                                        &score_tmp, &pose_estimate_tmp))
            continue;
        else{
          if(score_tmp > score){
            score = score_tmp;
            pose_estimate = pose_estimate_tmp;
          }
        }
      }
      gettimeofday(&tpend,NULL);
      timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
      LOG(INFO)<<"Lower resolution submap matching used "<<timeuse<<"s.";

      if(score>=_score_thresh_lower){
          score_tmp = 0.0; //reset score and score_tmp.
          score = 0.0;
          std::vector<int> index_vec = getSubmapCandidates(pose_estimate);
//          writeSubmaps(index_vec);//for debugging.
          LOG(INFO)<<"filtered submaps' size is "<<index_vec.size();
          gettimeofday(&tpstart,NULL);
          for(const int& ind: index_vec){
            if(!_submap_scan_matchers_higher[ind]->_scan_matcher_ptr->
                    MatchFullSubmap(point_cloud, kMinScore,&score_tmp, &pose_estimate_tmp))
                continue;
            else{
              if(score_tmp > score){
                score = score_tmp;
                pose_estimate = pose_estimate_tmp;
              }
            }
          }
          gettimeofday(&tpend,NULL);
          timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
          LOG(INFO)<<"Higher resolution submap matching used "<<timeuse<<"s.";
          if(score>_score_thresh_higher){
              res.x = pose_estimate.translation().coeff(0,0);;
              res.y = pose_estimate.translation().coeff(1,0);
              res.theta = pose_estimate.rotation().angle();
              return true;
          }else{
              return false;
          }
      }
      else{
          return false;
      }
  }
  return false;
}

bool GlobalLocator::match(const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg, GlobalPose2D &res){
  //todo,wz. align MultiEchoLaserScan with imu first then call scanmatcher's match function.
  LOG(ERROR)<<"Not implemented yet!";
  return false;
}

::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
GlobalLocator::loadOptions(const std::string cfg_file_dir,const std::string cfg_file_name){
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

std::vector<int> GlobalLocator:: getSubmapCandidates(const cartographer::transform::Rigid2d &pose_in_lower_res_map){
    if(_submap_scan_matchers_higher.empty()){
        return std::vector<int>();
    }
    std::vector<int> index_vec;
    for(int i = 0; i < _submap_scan_matchers_higher.size();++i){
        if(distance(_submap_scan_matchers_higher[i]->_center, pose_in_lower_res_map)
                <_distance_thresh){
           index_vec.push_back(i);
        }
    }
    return index_vec;
}

int GlobalLocator::getNearestSubmap(const cartographer::transform::Rigid2d &pose_in_histmap){
  if(_submap_scan_matchers_higher.empty()){
      return -1;
  }
  int min_dist_index = 0;
  float min_dist = 99999.;
  for(int i = 0; i < _submap_scan_matchers_higher.size();++i){
    float dist = distance(_submap_scan_matchers_higher[i]->_center, pose_in_histmap);
      if(dist<min_dist){
         min_dist = dist;
         min_dist_index = i;
      }
  }
  return min_dist_index;
}

double GlobalLocator::distance(const cartographer::transform::Rigid2d &pose1,
                               const cartographer::transform::Rigid2d &pose2){
    double x1 = pose1.translation().coeff(0,0);
    double y1 = pose1.translation().coeff(1,0);
    double x2 = pose2.translation().coeff(0,0);
    double y2 = pose2.translation().coeff(1,0);
    return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


void GlobalLocator::writeSubmaps(const std::vector<int> &index){
    std::string map_filestem = "/home/wz/filtered-submap";
    double resolution = 0.05;

    ::cartographer::io::ProtoStreamReader reader(_pb_filename);
    ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

    const auto& pose_graph = deserializer.pose_graph();

    LOG(INFO) << "Loading submap slices from serialized data.";
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        submap_slices;
    ::cartographer::mapping::proto::SerializedData proto;
    int i = -1;
    while (deserializer.ReadNextSerializedData(&proto)) {
      if (proto.has_submap()) {
        i+=1;
        const auto& submap = proto.submap();
        auto it = std::find(index.begin(), index.end(), i);
        if(it==index.end()) continue;
        const ::cartographer::mapping::SubmapId id{
            submap.submap_id().trajectory_id(),
            submap.submap_id().submap_index()};
        const ::cartographer::transform::Rigid3d global_submap_pose =
            ::cartographer::transform::ToRigid3(
                pose_graph.trajectory(id.trajectory_id)
                    .submap(id.submap_index)
                    .pose());

        FillSubmapSlice(global_submap_pose, submap, &submap_slices[id]);
      }
    }
    CHECK(reader.eof());

    LOG(INFO) << "Generating combined map image from submap slices.";
    auto result =
        ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

    ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

    ::cartographer::io::Image image(std::move(result.surface));
    cartographer_ros::WritePgm(image, resolution, &pgm_writer);

    const Eigen::Vector2d origin(
        -result.origin.x() * resolution,
        (result.origin.y() - image.height()) * resolution);

    ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
    cartographer_ros::WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}
