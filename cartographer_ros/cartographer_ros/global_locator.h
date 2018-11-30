#ifndef GLOBAL_LOCATOR_H
#define GLOBAL_LOCATOR_H

#include <string>
#include <vector>
#include <memory>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"

#include "distance_map_matcher.h"

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
                        ::cartographer::transform::Rigid2d origin,
                        ::cartographer::transform::Rigid2d center){
          _scan_matcher_ptr = scan_matcher_ptr;
          _origin = origin;
          _center = center;
      }
      std::shared_ptr<::cartographer::mapping::scan_matching
                      ::FastCorrelativeScanMatcher2D> _scan_matcher_ptr;
      ::cartographer::transform::Rigid2d _origin;
      ::cartographer::transform::Rigid2d _center;
  };

  GlobalLocator() = delete;
  ////locate using single resolution.
  GlobalLocator(const std::string& pbfilepath);

  ////locate using high and low resolution maps.
  GlobalLocator(const std::string& high_resolution_pbfilepath,
                const std::string& low_resolution_pbfilepath);

  bool match(const sensor_msgs::LaserScan::ConstPtr &msg, GlobalPose2D &res);
  bool match(const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg, GlobalPose2D &res);

  //temp interface, for experiment.
  void loadHistMap(const std::string& hist_file_path);
  bool matchWithHistmap(const sensor_msgs::LaserScan::ConstPtr &msg, GlobalPose2D &res);
private:
  void loadSubmaps(const std::string& pbfilepath,
                    std::vector<std::shared_ptr<SubmapScanMatcher>>& matcher_vec_ptr);

  //just for experiment.remove if not need any more.
  void writeSubmaps(const std::vector<int>& index);

  std::vector<int> getSubmapCandidates(const cartographer::transform::Rigid2d& pose_in_lower_res_map);
  int getNearestSubmap(const cartographer::transform::Rigid2d& pose_in_histmap);

  double distance(const cartographer::transform::Rigid2d& pose1,
                  const cartographer::transform::Rigid2d& pose2);
  ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
  loadOptions(const std::string cfg_file_dir,const std::string cfg_file_name);

  bool _inited = false;
  bool _two_stage_mode = false;

  //match score threshes to diffrent resolution submaps.
  float _score_thresh_higher = 0.7;
  float _score_thresh_lower = 0.5;

  //how close of a higher resolution submap to best matched lower resolution  submap (m).
  float _distance_thresh = 5;

  std::string _pb_filename;

  std::vector<std::shared_ptr<SubmapScanMatcher>> _submap_scan_matchers_higher;
  std::vector<std::shared_ptr<SubmapScanMatcher>> _submap_scan_matchers_lower;

  DistanceMapMatcher _hist_matcher;
};

#endif // GLOBAL_LOCATOR_H
