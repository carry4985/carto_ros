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
#include <fstream>

#include "global_locator.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include "node_constants.h"

#include "distance_map.h"
#include "distance_map_matcher.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"

#include "time_conversion.h"

DEFINE_string(pb_higher_res, "",
              "Filename of a high resolution pbstream to load.");
DEFINE_string(pb_lower_res, "",
              "Filename of a lower resolution pbstream to load.");
DEFINE_string(map_dir,"/home/wz/ROS/carto_ws/bag_map_pb/map", "dir where save map.pgm and map.yaml.");

DEFINE_string(bag_filename,"", "rosbag to play.");
namespace cartographer_ros {
namespace {

void splitString(const std::string& s, std::vector<std::string>& v, const std::string& c){
    using namespace std;
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void trans_tf(const GlobalLocator::GlobalPose2D&pose,
              geometry_msgs::TransformStamped&transform){
    transform.header.frame_id = "map";
    transform.child_frame_id = "laser";
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.translation.z = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond(
          Eigen::AngleAxisd(pose.theta, Eigen::Vector3d(0,0,1)) );
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
}

void readGroundTruthInfo(const std::string& path,std::vector<std::vector<std::string>>& grd_truth_info){
   std::ifstream ifs(path);
   if(!ifs){
     LOG(ERROR)<<"open file failed!";
     return;
   }

   std::string line;
   int line_id = 0;
   while(std::getline(ifs, line)){
     if(line.empty())break;
     //if(++line_id%10 != 0) continue;

     std::vector<std::string> str_vec;
     splitString(line,str_vec,",");
     grd_truth_info.push_back(str_vec);
   }
   ifs.close();
}

void Run(const std::shared_ptr<GlobalLocator>& locator) {
  //locator->loadHistMap("/home/wz/ROS/carto_ws/bag_map_pb/map/hist.txt");
  ::ros::NodeHandle nh;
  ::ros::Subscriber laser_scan_subscriber;

  bool kLocated = false;
  constexpr int localInfiniteSubscriberQueueSize = 1;
  sensor_msgs::LaserScan msgToMatch;

  static GlobalLocator::GlobalPose2D tmpRes;
  laser_scan_subscriber = nh.subscribe(
          kLaserScanTopic, localInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
          [&](const sensor_msgs::LaserScan::ConstPtr& msg){
              struct timeval tpstart,tpend;
              float timeuse;
              gettimeofday(&tpstart,NULL);

//              if(locator->matchWithHistmap(msg, tmpRes)){
              locator->match(msg, tmpRes);
              if(tmpRes.prob>0.7){
                  LOG(INFO)<<"Matched location is (<<"<<tmpRes.x<<","<<tmpRes.y<<","<<tmpRes.theta<<")";
                  kLocated = true;
                  msgToMatch = *msg;
              }
              gettimeofday(&tpend,NULL);
              timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
              LOG(INFO) << "All Use time:" << timeuse;
          })
  );

  while(::ros::ok()){
    ::ros::spinOnce();
    if(kLocated){
      //visulizing for debugging.
      //once located, shutdown subscriber.then get into tf broadcasting loop.
      laser_scan_subscriber.shutdown();
      static tf2_ros::TransformBroadcaster br;
      ::ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("matched_laser_scan", 100);
      ros::Rate loop_rate(10);

      geometry_msgs::TransformStamped transform;
      trans_tf(tmpRes, transform);
      while(::ros::ok()){
        auto now = ros::Time::now();
        transform.header.stamp = now;
        msgToMatch.header.stamp = now;
        scan_pub.publish(msgToMatch);
        br.sendTransform(transform);
        loop_rate.sleep();
        ::ros::spinOnce();
      }
    }
  }
}

void testDistanceMap(std::string yamldir){
//    DistanceMap distmap(yamldir);
//    distmap.computeHistgramForMap();
    ::ros::NodeHandle nh;
    ::ros::Subscriber laser_scan_subscriber;
    bool kLocated = false;
    constexpr int localInfiniteSubscriberQueueSize = 1;
    DistanceMapMatcher distmap_matcher;
//    distmap_matcher.loadDistanceMapData("/home/wz/ROS/carto_ws/bag_map_pb/map/DistMaps.txt",
//                                        "/home/wz/ROS/carto_ws/bag_map_pb/map/IdxOfMinDist.txt");
    distmap_matcher.loadHistgramMapData("/home/wz/ROS/carto_ws/bag_map_pb/map/hist.txt");
    laser_scan_subscriber = nh.subscribe(
            kLaserScanTopic, localInfiniteSubscriberQueueSize,
            boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [&](const sensor_msgs::LaserScan::ConstPtr& msg){
                struct timeval tpstart,tpend;
                float timeuse;
                gettimeofday(&tpstart,NULL);

                cv::Mat m = distmap_matcher.getCandidates("/home/wz/ROS/carto_ws/bag_map_pb/map/map.pgm",msg);
                gettimeofday(&tpend,NULL);
                timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
                LOG(INFO) << "All Use time:" << timeuse;

                 cv::imwrite("/home/wz/test.jpg", m);
                 kLocated = true;
                 LOG(INFO)<<"ok!";
            })
    );

    while(::ros::ok()){
      ::ros::spinOnce();
      if(kLocated){
        laser_scan_subscriber.shutdown();
        break;
      }
    }
}

//match with sampled scans along the trajectory. write results for paper.
void matchAllScans(const std::shared_ptr<GlobalLocator>& locator){
    locator->loadHistMap("/home/wz/ROS/carto_ws/bag_map_pb/map/hist.txt");
    std::vector<std::vector<std::string>> grd_truth_info;
    readGroundTruthInfo("/home/wz/ROS/carto_ws/nodes_info.txt",grd_truth_info);

    rosbag::Bag bag;
    bag.open(FLAGS_bag_filename);
    std::ofstream ofs("match_result_stats.txt");
    if(!ofs){
      LOG(ERROR)<<"open stats file failed!";
      return;
    }

    int64 min_secs = 99999;
    GlobalLocator::GlobalPose2D pyramid_res, histgram_res;
    struct timeval tpstart,tpend;
    float timeuse_pyramid, timeuse_histgram;

    for(int i = 0; i<grd_truth_info.size();i++){
      sensor_msgs::LaserScan::ConstPtr nearest_scan;
      //found nearest message by time.
      for(rosbag::MessageInstance const msg: rosbag::View(bag)){
        const sensor_msgs::LaserScan::ConstPtr scan = msg.instantiate<sensor_msgs::LaserScan>();
        if (scan != NULL){
            auto timestamp_ros = scan->header.stamp;
            int64 timestamp_carto = cartographer::common::ToUniversal(FromRos(timestamp_ros));
            if(abs(timestamp_carto - atoi(grd_truth_info[i][0].c_str())) < min_secs){
              min_secs = abs(timestamp_carto - atoi(grd_truth_info[i][0].c_str()));
              nearest_scan = msg.instantiate<sensor_msgs::LaserScan>();
           }
        }
      }

      LOG(INFO)<<"gap in nsec: "<<min_secs;
      min_secs = 99999;//reset.

      //mode 0, use multi resolution map.
      gettimeofday(&tpstart,NULL);
      locator->match(nearest_scan, pyramid_res);
      gettimeofday(&tpend,NULL);
      timeuse_pyramid=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

      //mode 1, use histgram map.
      gettimeofday(&tpstart,NULL);
      locator->matchWithHistmap(nearest_scan, histgram_res);
      gettimeofday(&tpend,NULL);
      timeuse_histgram=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

      ofs<<grd_truth_info[i][0]<<","<<grd_truth_info[i][1]<<","
           <<grd_truth_info[i][2]<<","<<grd_truth_info[i][3]<<","
           <<grd_truth_info[i][4]<<","<<
           pyramid_res.x<<","<<pyramid_res.y<<","<<pyramid_res.theta<<","<<pyramid_res.prob<<","<<timeuse_pyramid<<","<<
           histgram_res.x<<","<<histgram_res.y<<","<<histgram_res.theta<<","<<histgram_res.prob<<","<<timeuse_histgram<<",\n";
      LOG(INFO)<<"Scan with time stamp "<<grd_truth_info[i][1]<<" matched successfully!";
    }
    ofs.close();
    bag.close();
}

//only match with scans whose timestamp in fitered_match_res_table.txt.
//just for writing paper. remove it if not need any more.
void matchAllScansLowerResolution(const std::shared_ptr<GlobalLocator>& locator){
    std::vector<std::vector<std::string>> grd_truth_info;
    readGroundTruthInfo("/home/wz/Desktop/fitered_match_res_table.txt",grd_truth_info);

    rosbag::Bag bag;
    bag.open(FLAGS_bag_filename);
    std::ofstream ofs("match_result_stats_lower.txt");
    if(!ofs){
      LOG(ERROR)<<"open stats file failed!";
      return;
    }

    int64 min_secs = 99999;
    GlobalLocator::GlobalPose2D res;
    struct timeval tpstart,tpend;
    float timeuse;

    for(int i = 0; i<grd_truth_info.size();i++){
      sensor_msgs::LaserScan::ConstPtr nearest_scan;
      //found nearest message by time stamp.
      for(rosbag::MessageInstance const msg: rosbag::View(bag)){
        const sensor_msgs::LaserScan::ConstPtr scan = msg.instantiate<sensor_msgs::LaserScan>();
        if (scan != NULL){
            auto timestamp_ros = scan->header.stamp;
            int64 timestamp_carto = cartographer::common::ToUniversal(FromRos(timestamp_ros));

            if(abs(timestamp_carto - atoi(grd_truth_info[i][0].c_str())) < min_secs){
              min_secs = abs(timestamp_carto - atoi(grd_truth_info[i][0].c_str()));
              nearest_scan = msg.instantiate<sensor_msgs::LaserScan>();
           }
        }
      }
      min_secs = 99999;//reset.

      //mode 0, use lower resolution map.
      gettimeofday(&tpstart, NULL);
      locator->match(nearest_scan, res);
      gettimeofday(&tpend, NULL);
      timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

      ofs<<grd_truth_info[i][0]<<","<<grd_truth_info[i][1]<<","
           <<grd_truth_info[i][2]<<","<<grd_truth_info[i][3]<<","
           <<grd_truth_info[i][4]<<","<<
           res.x<<","<<res.y<<","<<res.theta<<","<<res.prob<<","<<timeuse<<",\n";
      LOG(INFO)<<i<<", Scan with time stamp "<<grd_truth_info[i][1]<<" matched successfully!";
    }
    ofs.close();
    bag.close();
}

void showSelectedPoints(){
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  std::string info_file_path = "/home/wz/Desktop/match_res_table.txt";
  std::vector<std::vector<std::string>> matched_info_vec;
  readGroundTruthInfo( info_file_path, matched_info_vec);
  ros::Rate r(30);

  visualization_msgs::Marker points_in_graph, points_pyramid, points_histgram;
  points_in_graph.header.frame_id = points_pyramid.header.frame_id = points_histgram.header.frame_id="/map";
  points_in_graph.header.stamp = points_pyramid.header.stamp = points_histgram.header.stamp=ros::Time::now();
  points_in_graph.ns = "points_in_graph";
  points_pyramid.ns = "points_pyramid";
  points_histgram.ns = "points_histgram";

  points_in_graph.action = points_pyramid.action=points_histgram.action=visualization_msgs::Marker::ADD;
  points_in_graph.pose.orientation.w = points_pyramid.pose.orientation.w=points_histgram.pose.orientation.w=1.0;
  points_in_graph.id = 0;
  points_pyramid.id = 1;
  points_histgram.id = 2;
  points_in_graph.type =points_pyramid.type= points_histgram.type=visualization_msgs::Marker::POINTS;
  points_in_graph.scale.x = points_pyramid.scale.x=points_histgram.scale.x=0.2;
  points_in_graph.scale.y = points_pyramid.scale.y=points_histgram.scale.y=0.2;
  points_in_graph.color.g = 1.0f;
  points_pyramid.color.r = 0.5f;
  points_histgram.color.b = 0.5f;

  points_in_graph.color.a = points_pyramid.color.a = points_histgram.color.a = 1.0f;
  for (int i = 0; i < matched_info_vec.size(); ++i){
    geometry_msgs::Point p1,p2,p3;
    p1.x = atof(matched_info_vec[i][2].c_str());
    p1.y = atof(matched_info_vec[i][3].c_str());
    p2.x = atof(matched_info_vec[i][5].c_str());
    p2.y = atof(matched_info_vec[i][6].c_str());
    p3.x = atof(matched_info_vec[i][10].c_str());
    p3.y = atof(matched_info_vec[i][11].c_str());
    points_in_graph.points.push_back(p1);
    points_pyramid.points.push_back(p2);
    points_histgram.points.push_back(p3);
  }

  while (ros::ok()){
    marker_pub.publish(points_in_graph);
    marker_pub.publish(points_pyramid);
    marker_pub.publish(points_histgram);
    r.sleep();
  }
}
}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);


  CHECK(!FLAGS_pb_higher_res.empty()) << "-pb_higher_res is missing.";
  ::ros::init(argc, argv, "cartographer_global_localization");
  ::ros::start();

  std::shared_ptr<GlobalLocator> locator;
  if(!FLAGS_pb_lower_res.empty()){
      locator = std::make_shared<GlobalLocator>(FLAGS_pb_higher_res,FLAGS_pb_lower_res);
  } else {
    locator =std::make_shared<GlobalLocator>(FLAGS_pb_higher_res);
  }
//  ::cartographer_ros::matchAllScans(locator);
  ::cartographer_ros::matchAllScansLowerResolution(locator);

//  ::cartographer_ros::Run(locator);
//  ::cartographer_ros::testDistanceMap(FLAGS_map_dir);

//    ::cartographer_ros::showSelectedPoints();

  ::ros::shutdown();
}
