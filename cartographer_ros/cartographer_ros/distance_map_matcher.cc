#include "distance_map_matcher.h"
#include <fstream>
#include <cmath>
#include "glog/logging.h"

DistanceMapMatcher::DistanceMapMatcher(std::string DistMapsPath,
                                       std::string MinIdxListPath)
        : DistMapsPath(DistMapsPath),
          MinIdxListPath(MinIdxListPath)
{
    std::ifstream fin(DistMapsPath.c_str());
    if(!fin)
        LOG(INFO) << "DistMapsFile does not exhist!!!";

    float maxValInMin=0.0, maxValInMax=0.0,maxValInAvg=0.0,maxValInVar=0.0;
    while(!fin.eof())
    {
        MinMaxAlphaDist tempMinMaxAlphaDist_;
        std::string str;
        getline(fin,str);
        int pos=str.find("=");

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="x")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.x=atof(temp.c_str());
        }

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="y")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.y=atof(temp.c_str());
        }
        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="col")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.col=(int)atof(temp.c_str());
        }

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="row")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.row=(int)atof(temp.c_str());
        }

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="mindist")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.mindist=atof(temp.c_str());
        }

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="maxdist")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.maxdist=atof(temp.c_str());
        }

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="avgdist")
        {
            std::string temp = str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.avgdist=atof(temp.c_str());
        }

        str.clear();
        getline(fin,str);
        pos=str.find("=");
        if(str.substr(0,pos)=="variance")
        {
            std::string temp=str.substr(pos+1,str.length()-pos-1);
            tempMinMaxAlphaDist_.variance=atof(temp.c_str());
        }

        VectorMinMaxAlphaDist.push_back(tempMinMaxAlphaDist_);
        maxValInMin=tempMinMaxAlphaDist_.mindist>maxValInMin ? tempMinMaxAlphaDist_.mindist:maxValInMin;
        maxValInMax=tempMinMaxAlphaDist_.maxdist>maxValInMax ? tempMinMaxAlphaDist_.maxdist:maxValInMax;
        maxValInAvg=tempMinMaxAlphaDist_.avgdist>maxValInAvg ? tempMinMaxAlphaDist_.avgdist:maxValInAvg;
        maxValInVar=tempMinMaxAlphaDist_.variance>maxValInVar? tempMinMaxAlphaDist_.variance:maxValInVar;
    }

    _featureMaxes = {maxValInMin,maxValInMax,maxValInAvg,maxValInVar};
    /*------------ read VectorIdx of mindist------------*/

    std::ifstream fin_minidxlist(MinIdxListPath.c_str());
    if(!fin_minidxlist)
        LOG(INFO) << "VectorIdx file of mindist does not exhist!!!" ;

    fin_minidxlist >> origin[0] >> origin[1] >> origin[2];
    fin_minidxlist.close();

    fin_minidxlist.open(MinIdxListPath.c_str());
    std::string tmpStr;
    getline(fin_minidxlist, tmpStr);

    int iii = 0;
    while(!fin_minidxlist.eof())
    {
        std::string str;
        getline(fin_minidxlist,str);
        int pos=str.find("=");
        std::string temp=str.substr(pos+1,str.length()-pos-1);

        int minidx=-1;
        minidx=(int)atof(temp.c_str());

        IdxOfMinDistVector.push_back(minidx);
        iii+=1;
    }
}

cv::Mat DistanceMapMatcher::getCandidates(const std::string &pgmpath,
                                          const sensor_msgs::LaserScan::ConstPtr &msg){
    cv::Mat map = cv::imread(pgmpath,1);
    auto features = computeFeaturesForScan(msg);

//    std::vector<MinMaxAlphaDist> res;
//    bool ok = filterMinMaxAlphaDistMap(res,map,features[0],features[1],
//                                      features[2],features[3],0.5,0.5,2);
//    if(ok){
//         return map;
//    }
//    else{
//        return cv::Mat();
//    }
    std::vector<PixelWithCoef> res;
    getCoefVec(features, res);
    std::sort(res.begin(), res.end(), [](const PixelWithCoef& a,const PixelWithCoef& b){
        return a._coef > b._coef;
    });
    for(int i = 0; i < res.size(); i++ ){
        if(res[i]._coef>0.99){
            LOG(INFO)<<i<<","<<res[i]._coef;
            cv::circle(map, cv::Point(res[i]._col,res[i]._row),5,cv::Scalar(0,0,255));
        }else{
            break;
        }
    }
    return map;
}

void DistanceMapMatcher::VisitandFilter(BinTree *bTree,float key,float tolerance)
{
    if(bTree != NULL)
    {
        float mindist=(float)VectorMinMaxAlphaDist[bTree->idx].mindist;

        if (fabs(mindist-key)<tolerance)
        {
            //cout<<"push_back:"<<bTree->idx<<endl;
            FilteredVectorList.push_back(bTree->idx);
        }
        if(key<(float)VectorMinMaxAlphaDist[bTree->idx].mindist+tolerance)
        {
            VisitandFilter(bTree->left,key,tolerance);
        }
        if(key>(float)VectorMinMaxAlphaDist[bTree->idx].mindist-tolerance)
        {
            VisitandFilter(bTree->right,key,tolerance);
        }
    }
}

std::vector<float> DistanceMapMatcher::computeFeaturesForScan(const sensor_msgs::LaserScan::ConstPtr &msg){
    int length = 360;
    float rangeMax = msg->range_max;
    float rangeMin = msg->range_min;

    float minScan = 10000.;
    float maxScan = 0.;
    float sumScanDist = 0.;
    for(int i = 0; i < length; ++i)
    {
        float tmp = msg->ranges[i];
        if(tmp != INFINITY && tmp < rangeMax && tmp > rangeMin)
        {
            sumScanDist += tmp;
            if(tmp < minScan)
                minScan = tmp;
            if(tmp > maxScan)
                maxScan = tmp;
        }
    }

    float avgDist = sumScanDist / length;
    float sumVariance = 0.;
    for(int i = 0; i < length; ++i)
    {
        float tmp = msg->ranges[i];
        if(tmp != INFINITY && tmp < rangeMax && tmp > rangeMin)
        {
            sumVariance += (tmp - avgDist) * (tmp - avgDist);
        }
    }
    sumVariance = sumVariance / length;

    std::vector<float> attr = {minScan, maxScan, avgDist, sumVariance};
    return attr;
}

float DistanceMapMatcher::computeCoef(const std::vector<float>&vec1,
                                      const std::vector<float>&vec2){
    return (vec1[0]*vec2[0]+vec1[1]*vec2[1]+
            vec1[2]*vec2[2]+vec1[3]*vec2[3]) * 1.0 /
            (std::sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1]+
            vec1[2]*vec1[2]+vec1[3]*vec1[3])
            *std::sqrt(vec2[0]*vec2[0]+vec2[1]*vec2[1]+
            vec2[2]*vec2[2]+vec2[3]*vec2[3]));
}

void DistanceMapMatcher::getCoefVec(const std::vector<float>&scanFeatureVec,
                                    std::vector<PixelWithCoef>&coefVec){
    if(!coefVec.empty())coefVec.clear();
    std::vector<float> vec2(4),vec2Normed(4), scanFeatNormed;
    scanFeatNormed = normalize(scanFeatureVec);

    for (int idx=0; idx<VectorMinMaxAlphaDist.size(); idx++){
        int col=VectorMinMaxAlphaDist[idx].col;
        int row=VectorMinMaxAlphaDist[idx].row;
        float target_mindist=VectorMinMaxAlphaDist[idx].mindist;
        float target_maxdist=VectorMinMaxAlphaDist[idx].maxdist;
        float target_avgdist=VectorMinMaxAlphaDist[idx].avgdist;
        float target_variance=VectorMinMaxAlphaDist[idx].variance;
        vec2 = {target_mindist,target_maxdist,target_avgdist,target_variance};
        vec2Normed = normalize(vec2);
        float coef = computeCoef(scanFeatNormed, vec2Normed);
        coefVec.push_back(PixelWithCoef(row,col,coef));
    }
}

std::vector<float> DistanceMapMatcher::normalize(const std::vector<float>&featureVec){
    std::vector<float> normedFeatures = {featureVec[0]-_featureMaxes[0],
                              featureVec[1]-_featureMaxes[1],
                              featureVec[2]-_featureMaxes[2],
                              featureVec[3]-_featureMaxes[3]};
    return normedFeatures;
}

void DistanceMapMatcher::establishTree()
{
    root =NULL;

    int startidx=0;
    int endidx=IdxOfMinDistVector.size()-1;
    InsertFromVector(root,startidx,endidx);
}

void DistanceMapMatcher::InsertFromVector(BinTree*& root,int start,int end)
{
     if(start >end)
       return ;
     root = new BinTree;
     root->left = NULL;
     root->right = NULL;
     int mid = start+(end-start)/2;
     root->idx = IdxOfMinDistVector[mid];
     InsertFromVector(root->left,start,mid-1);
     InsertFromVector(root->right,mid+1,end);
}

bool DistanceMapMatcher::filterMinMaxAlphaDistMap(std::vector<MinMaxAlphaDist>& filteredVectorMinMaxAlphaDist,
                                                  cv::Mat& visMat,
                                                  float interest_min_dist,
                                                  float interest_max_dist,
                                                  float interest_avg_dist,
                                                  float interest_variance,
                                                  float buffer,
                                                  float buffer_variance,
                                                  int mode) {
    //method without the binTree search
    if(visMat.channels()!=3) {
        cv::cvtColor(visMat,visMat,cv::COLOR_GRAY2BGR);
    }
    if(mode==1) {
        LOG(INFO) << "using method 1";
        LOG(INFO) << "size of 'VectorMinMaxAlphaDist' is " << VectorMinMaxAlphaDist.size();

        bool getfilteredMap=false;

        int pixelcount_with_minmax_dist=0;
        int pixelcount_with_minmaxavg_dist=0;
        int pixelcount_with_minmaxavgvariance_dist=0;

        for (int idx=0; idx<VectorMinMaxAlphaDist.size(); idx++){
            int col=VectorMinMaxAlphaDist[idx].col;
            int row=VectorMinMaxAlphaDist[idx].row;
            float target_mindist=VectorMinMaxAlphaDist[idx].mindist;
            float target_maxdist=VectorMinMaxAlphaDist[idx].maxdist;
            float target_avgdist=VectorMinMaxAlphaDist[idx].avgdist;
            float target_variance=VectorMinMaxAlphaDist[idx].variance;

            if((float)interest_min_dist<target_mindist+buffer
                    &&(float)interest_min_dist>target_mindist-buffer){
                if((float)interest_max_dist<target_maxdist+buffer
                        &&(float)interest_max_dist>target_maxdist-buffer){
                    pixelcount_with_minmax_dist+=1;
                    if((float)fabs(interest_avg_dist-target_avgdist)<buffer){
                        pixelcount_with_minmaxavg_dist+=1;
                        if((float)fabs(interest_variance-target_variance)<buffer_variance){
                            pixelcount_with_minmaxavgvariance_dist+=1;

                            getfilteredMap=true;

                            MinMaxAlphaDist tempMinMaxAlphaDist;
                            tempMinMaxAlphaDist.y=VectorMinMaxAlphaDist[idx].y;
                            tempMinMaxAlphaDist.x=VectorMinMaxAlphaDist[idx].x;
                            tempMinMaxAlphaDist.col=VectorMinMaxAlphaDist[idx].col;
                            tempMinMaxAlphaDist.row=VectorMinMaxAlphaDist[idx].row;
                            tempMinMaxAlphaDist.mindist=VectorMinMaxAlphaDist[idx].mindist;
                            tempMinMaxAlphaDist.maxdist=VectorMinMaxAlphaDist[idx].maxdist;

                            tempMinMaxAlphaDist.avgdist=VectorMinMaxAlphaDist[idx].avgdist;
                            tempMinMaxAlphaDist.variance=VectorMinMaxAlphaDist[idx].variance;

                            filteredVectorMinMaxAlphaDist.push_back(tempMinMaxAlphaDist);
                            visMat.at<cv::Vec3b>(row,col)[0]=0;
                            visMat.at<cv::Vec3b>(row,col)[1]=0;
                            visMat.at<cv::Vec3b>(row,col)[2]=255;
                            cv::circle(visMat, cv::Point(col,row),5,cv::Scalar(0,0,255));
                        }
                    }
                }
            }
        }

        LOG(INFO)<<"size of pixels after minmax dist filtering : "<<pixelcount_with_minmax_dist;
        LOG(INFO)<<"size of pixels after minmaxavg dist filtering : "<<pixelcount_with_minmaxavg_dist;
        LOG(INFO)<<"size of pixels after minmaxavgvariance dist filtering : "<<pixelcount_with_minmaxavgvariance_dist;
        return getfilteredMap;
    }

    //method with the binTree search
    if(mode==2){
        LOG(INFO)<<"using method 2";
        LOG(INFO)<<"size of 'VectorMinMaxAlphaDist' is "<<VectorMinMaxAlphaDist.size();
        establishTree();
        BinTree* root=getTree();
        VisitandFilter(root,interest_min_dist,buffer);
        std::cout<<"size of 'FilteredVectorList' is "<<FilteredVectorList.size()<<std::endl;

        bool getfilteredMap=false;
        int pixelcount_with_minmax_dist=0;
        int pixelcount_with_minmaxavg_dist=0;
        int pixelcount_with_minmaxavgvariance_dist=0;

        for (int idx=0;idx<FilteredVectorList.size();idx++){
            int candidateidx=FilteredVectorList[idx];
            int col=VectorMinMaxAlphaDist[candidateidx].col;
            int row=VectorMinMaxAlphaDist[candidateidx].row;
            float target_maxdist=VectorMinMaxAlphaDist[candidateidx].maxdist;
            float target_avgdist=VectorMinMaxAlphaDist[candidateidx].avgdist;
            float target_variance=VectorMinMaxAlphaDist[candidateidx].variance;

            if((float)interest_max_dist<target_maxdist+buffer
                    &&(float)interest_max_dist>target_maxdist-buffer){
                pixelcount_with_minmax_dist+=1;

                if((float)interest_avg_dist<target_avgdist+buffer
                        &&(float)interest_avg_dist>target_avgdist-buffer){
                    pixelcount_with_minmaxavg_dist+=1;
                    if((float)fabs(interest_variance-target_variance)<buffer_variance){
                        pixelcount_with_minmaxavgvariance_dist+=1;
                        getfilteredMap=true;
                        MinMaxAlphaDist tempMinMaxAlphaDist;
                        tempMinMaxAlphaDist.y=VectorMinMaxAlphaDist[candidateidx].y;
                        tempMinMaxAlphaDist.x=VectorMinMaxAlphaDist[candidateidx].x;
                        tempMinMaxAlphaDist.col=VectorMinMaxAlphaDist[candidateidx].col;
                        tempMinMaxAlphaDist.row=VectorMinMaxAlphaDist[candidateidx].row;
                        tempMinMaxAlphaDist.mindist=VectorMinMaxAlphaDist[candidateidx].mindist;
                        tempMinMaxAlphaDist.maxdist=VectorMinMaxAlphaDist[candidateidx].maxdist;
                        tempMinMaxAlphaDist.avgdist=VectorMinMaxAlphaDist[candidateidx].avgdist;
                        tempMinMaxAlphaDist.variance=VectorMinMaxAlphaDist[candidateidx].variance;
                        filteredVectorMinMaxAlphaDist.push_back(tempMinMaxAlphaDist);
                        visMat.at<cv::Vec3b>(row,col)[0]=0;
                        visMat.at<cv::Vec3b>(row,col)[1]=0;
                        visMat.at<cv::Vec3b>(row,col)[2]=255;
                        cv::circle(visMat, cv::Point(col,row),5,cv::Scalar(0,0,255));
                    }
                }
            }
        }
        LOG(INFO) << "size of pixels after minmax dist filtering : "<<pixelcount_with_minmax_dist << std::endl;
        LOG(INFO) << "size of pixels after minmaxavg dist filtering : "<<pixelcount_with_minmaxavg_dist << std::endl;
        LOG(INFO) << "size of pixels after minmaxavgvariance dist filtering : "<<pixelcount_with_minmaxavgvariance_dist << std::endl;
        return getfilteredMap;
    }
    return false;
}
