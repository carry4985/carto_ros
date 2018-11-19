#ifndef GLOBAL_LOCATOR_H
#define GLOBAL_LOCATOR_H

#include <string>
#include <vector>
#include <memory>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"


class GlobalLocator {
public:
  struct GlobalPose2D {
      double x, y;
      double theta;
  };
  struct SubmapScanMatcher{
  public:
      SubmapScanMatcher() = delete;
      SubmapScanMatcher(std::shared_ptr<::cartographer::mapping::scan_matching
                        ::FastCorrelativeScanMatcher2D> scan_matcher_ptr,
                        ::cartographer::transform::Rigid2d submap_pose){
          _scan_matcher_ptr = scan_matcher_ptr;
          _submap_pose = submap_pose;
      }

      std::shared_ptr<::cartographer::mapping::scan_matching
                      ::FastCorrelativeScanMatcher2D> _scan_matcher_ptr;
      ::cartographer::transform::Rigid2d _submap_pose;
  };
  GlobalLocator(){}
  void build_matchers_from_pbstream(const std::string& pbfilepath);
  bool match(const sensor_msgs::LaserScan::ConstPtr &msg, GlobalPose2D &res);
  bool match(const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg, GlobalPose2D &res);
private:
  ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
  load_options(const std::string cfg_file_dir,const std::string cfg_file_name);


  bool _inited = false;
  float _score_thresh = 0.7;
  std::vector<std::shared_ptr<SubmapScanMatcher>> _submap_scan_matchers;

};

#endif // GLOBAL_LOCATOR_H
