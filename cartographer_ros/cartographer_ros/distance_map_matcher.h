#ifndef DISTANCE_MAP_MATCHER_H
#define DISTANCE_MAP_MATCHER_H
#include <vector>
#include "distance_map.h"
#include "sensor_msgs/LaserScan.h"

typedef struct Bin_tree BinTree;
struct Bin_tree
{
    int idx;
    BinTree* right;
    BinTree* left;
};


class DistanceMapMatcher
{
public:
    DistanceMapMatcher(){}
    //read "vecofDistMap" and sorted idx of the "vecofDistMap"
    void loadDistanceMapData(std::string DistMapsPath, std::string MinIdxListPath);
    void loadHistgramMapData(const std::string& hist_file_path);
    //method without the binTree: select appropiate vector<MinMaxAlphaDist>
    bool filterMinMaxAlphaDistMap(std::vector<MinMaxAlphaDist>& filteredVectorMinMaxAlphaDist,
                                  cv::Mat& visMat,
                                  float interest_min_dist,
                                  float interest_max_dist,
                                  float interest_avg_dist,
                                  float interest_variance,
                                  float buffer,
                                  float buffer_variance,
                                  int mode);

    std::vector<cv::Point2f> getCandidates(const sensor_msgs::LaserScan::ConstPtr &msg, const int top_n);

    //for debug
    cv::Mat getCandidates(const std::string& pgmpath,
                          const sensor_msgs::LaserScan::ConstPtr &msg);
    float* getOrigin(){ return origin; }

private:
    struct PixelWithCoef{
        PixelWithCoef(int r,int c, float coef){
            _row = r;
            _col = c;
            _coef = coef;
        }
        int _row;
        int _col;
        float _coef;
    };
    enum HIST_COMPARE_MODE{CORREL=0,CHISQR=1,BHATTACHARYYA=2,};
    void establishTree();

    std::vector<float> computeFeaturesForScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    Histgram computeHistForScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::vector<float> normalize(const std::vector<float>& featureVec);

    void getCoefVecDist(const std::vector<float>& scanFeatureVec, std::vector<PixelWithCoef>&coefVec);
    void getCoefVecHist(const std::vector<int>& scanFeatureVec, std::vector<PixelWithCoef>&coefVec);

    float compareHist(const std::vector<int>& hist1, const std::vector<int>& hist2, HIST_COMPARE_MODE mode=CORREL);
    template<typename T>
    float computeCoef(const std::vector<T>& vec1, const std::vector<T>& vec2){
      float s11 = 0., s22 = 0., s12 = 0.;
      if(vec1.size()!=vec2.size() || vec1.empty()) return 0.0;
      for(int i = 0; i < vec1.size(); i++){
        s11 += vec1[i]*vec1[i];
        s22 += vec2[i]*vec2[i];
        s12 += vec1[i]*vec2[i];
      }
      return s12 * 1./ (std::sqrt(s11)*std::sqrt(s22));
    }

    void InsertFromVector(BinTree*&, int start, int end);
    BinTree* getTree() { return root; }

    //go through the tree and filter the Vector of mindist according to "key" and "tolerance"
    void VisitandFilter(BinTree *bTree, float key, float tolerance);

    std::vector<int> getFilteredVectorList() { return FilteredVectorList; }

    void splitString(const std::string& s, std::vector<std::string>& v, const std::string& c);
private:
    std::vector<MinMaxAlphaDist> VectorMinMaxAlphaDist;
    std::vector<int> IdxOfMinDistVector;
    std::vector<int> VecSortedIdx;
    std::string DistMapsPath, MinIdxListPath;

    //tree
    BinTree* root;
    //filtered vector of mindist
    std::vector<int> FilteredVectorList;
    //filtered vector according maxdist based on "FilteredVectorList"
    std::vector<int> FilteredVectorListFinal;
    float origin[3];

    //store the max feature in each dim for nomalization. added by wz.
    std::vector<float> _featureMaxes;
    std::vector<Histgram> _histgrams;
};

#endif // DISTANCE_MAP_MATCHER_H
