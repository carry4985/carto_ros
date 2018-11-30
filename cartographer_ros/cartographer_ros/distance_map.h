#ifndef DISTANCE_MAP_H
#define DISTANCE_MAP_H
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace params {
  const float g_min_scan_range = 0.15;
  const float g_max_scan_range = 12;
  const int g_size_of_bin = 100;
}

struct MinMaxAlphaDist{
    int col;
    int row;
    float x;
    float y;
    float mindist=9999;
    float maxdist=-1;
    float avgdist=-1;
    float variance=-1;
};

struct Histgram
{
  Histgram() = delete;
  Histgram(int x,int y, int size){
    _col = x;
    _row = y;
    _size_of_bins = size;
    _hist = std::vector<int>(size,0);
  }

  int _row = 0;
  int _col = 0;
  int _size_of_bins;
  std::vector<int> _hist;
};

//precompute this distance map for global localization.
class DistanceMap{
public:
    DistanceMap(std::string para_map_dir);
    //compute Min, Max Map for the pgm image
    bool computeMinMaxAlphaDistforMap();
    //wz
    bool computeHistgramForMap();
private:
    bool writeDistMap();
    bool writeHistMap();

    //Compute Min,Max distance for one pixel
    void computeMinMaxAlphaDist(MinMaxAlphaDist& MinMaxAlphaDist_,cv::Mat& img,
                                int interestP_x,int interestP_y,float SearchWindow_radius);

    Histgram computeHistAtPixel(const cv::Mat& img,const int x,const int y,
                                const float search_radius,int& valid_count);

private:
    std::vector<MinMaxAlphaDist> _vec_min_max_dist;
    std::map<std::string, std::string> _data;
    cv::Mat _img;
    float _origin[3];
    float _resolution;

    std::string _map_dir;
    const std::string _map_name = "map.pgm";
    const std::string _map_yaml = "map.yaml";
    const std::string _hist_file_name = "hist.txt";
    const std::string _dist_file_name = "dist.txt";

    //0.5-8m, every 0.5m a bin
    int _hist_size = params::g_size_of_bin;
    int _min_scan_len = params::g_min_scan_range;
    int _max_scan_len = params::g_max_scan_range;
    std::vector<Histgram> _histgrams;
};

#endif // DISTANCE_MAP_H
