#ifndef DISTANCE_MAP_H
#define DISTANCE_MAP_H
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct MinMaxAlphaDist
{
    int col;
    int row;
    float x;
    float y;
    float mindist=9999;
    float maxdist=-1;
    float avgdist=-1;
    float variance=-1;
};

//precompute this distance map for global localization.
class DistanceMap
{
public:
    DistanceMap(std::string para_YamlFileRootPath);
    //compute Min, Max Map for the pgm image
    bool ComputeMinMaxAlphaDistforimg(float SearchWindow_radius);
private:
    bool WriteDistMap();

    //Compute Min,Max distance for one pixel
    void ComputeMinMaxAlphaDist(MinMaxAlphaDist& MinMaxAlphaDist_,cv::Mat& img,
                                int interestP_x,int interestP_y,float SearchWindow_radius);

private:
    std::vector<MinMaxAlphaDist> VectorMinMaxAlphaDist;

    std::map<std::string, std::string> data;
    cv::Mat img;
    float origin[3];
    float resolution;

    std::string m_YamlFileRootPath;
};

#endif // DISTANCE_MAP_H
