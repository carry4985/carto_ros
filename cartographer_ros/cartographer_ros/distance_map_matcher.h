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
    //read "vecofDistMap" and sorted idx of the "vecofDistMap"
    DistanceMapMatcher(std::string DistMapsPath, std::string MinIdxListPath);

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

    cv::Mat getCandidates(const std::string& pgmpath,
                          const sensor_msgs::LaserScan::ConstPtr &msg);
    float* getOrigin(){ return origin; }

private:
    //method with the binTree
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
    void establishTree();

    std::vector<float> computeFeaturesForScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::vector<float> normalize(const std::vector<float>& featureVec);
    float computeCoef(const std::vector<float>& vec1, const std::vector<float>& vec2);
    void getCoefVec(const std::vector<float>& scanFeatureVec, std::vector<PixelWithCoef>&coefVec);

    void InsertFromVector(BinTree*&, int start, int end);
    BinTree* getTree() { return root; }

    //go through the tree and filter the Vector of mindist according to "key" and "tolerance"
    void VisitandFilter(BinTree *bTree, float key, float tolerance);

    std::vector<int> getFilteredVectorList() { return FilteredVectorList; }

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
};

#endif // DISTANCE_MAP_MATCHER_H

