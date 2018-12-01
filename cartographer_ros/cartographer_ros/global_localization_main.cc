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
              if(locator->match(msg, tmpRes)){
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
    rosbag::Bag bag;
    bag.open(FLAGS_bag_filename);
    std::ofstream ofs("match_result_stats.txt");
    if(!ofs){
      LOG(ERROR)<<"open stats file failed!";
      return;
    }

    const int kSkipSecs = 5;//match every 10s.
    int64 secs = 0;
    GlobalLocator::GlobalPose2D pyramid_res, histgram_res;
    struct timeval tpstart,tpend;
    float timeuse_pyramid, timeuse_histgram;
    for(rosbag::MessageInstance const msg: rosbag::View(bag)){
      const sensor_msgs::LaserScan::ConstPtr scan = msg.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL){
          auto timestamp_ros = scan->header.stamp;
          const int64 timestamp_carto =
              cartographer::common::ToUniversal(FromRos(timestamp_ros));

          if(secs==0){
            secs = timestamp_ros.sec;
          }
          if(secs + kSkipSecs > timestamp_ros.sec){
            continue;
          }else{
            secs = timestamp_ros.sec;
          }
          //mode 0, use multi resolution map.
          gettimeofday(&tpstart,NULL);
          bool is_ok_pyramid = locator->match(scan, pyramid_res);
          gettimeofday(&tpend,NULL);
          timeuse_pyramid=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

          //mode 1, use histgram map.
          gettimeofday(&tpstart,NULL);
          bool is_ok_histgram = locator->matchWithHistmap(scan, histgram_res);
          gettimeofday(&tpend,NULL);
          timeuse_histgram=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

          if(is_ok_histgram && is_ok_pyramid){
            ofs<<timestamp_carto<<","<<timestamp_ros<<","<<
                 pyramid_res.x<<","<<pyramid_res.y<<","<<pyramid_res.theta<<","<<timeuse_pyramid<<","<<
                 histgram_res.x<<","<<histgram_res.y<<","<<histgram_res.theta<<","<<timeuse_histgram<<"\n";
            LOG(INFO)<<"Scan with time stamp "<<timestamp_ros<<" matched successfully!";
          }
      }
    }
    ofs.close();
    bag.close();
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
  ::cartographer_ros::matchAllScans(locator);

//  ::cartographer_ros::Run(locator);
//  ::cartographer_ros::testDistanceMap(FLAGS_map_dir);

  ::ros::shutdown();
}
