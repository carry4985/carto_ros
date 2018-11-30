#include "distance_map.h"
#include <fstream>
#include <glog/logging.h>
DistanceMap::DistanceMap(std::string para_map_dir)
    :_map_dir(para_map_dir)
{
    std::ifstream fin( (_map_dir + "/" + _map_yaml).c_str() );
    if(!fin)
        std::cout << "yaml file does not exist!" << std::endl;

    while(!fin.eof())
    {
        std::string str;
        getline(fin,str);

        int pos = str.find(":");
        if (pos == -1)
            continue;
        std::string key = str.substr( 0, pos );
        std::string value = str.substr( pos+2, str.length()-pos-2 );
        _data[key] = value;

        if ( !fin.good() )
            break;
    }

    //resolution
    std::string resolution_=_data["resolution"];
    _resolution=atof(resolution_.c_str());

    std::string PgmFilePath = _map_dir + "/" + _map_name;
    _img=cv::imread(PgmFilePath, CV_LOAD_IMAGE_GRAYSCALE);

    //origin
    std::string origin_=_data["origin"];
    int first=origin_.find(",");
    std::string origin_x_=origin_.substr(1,first-1);
    origin_=origin_.substr(first+2,origin_.length()-first-2);
    int second=origin_.find(",");
    std::string origin_y_=origin_.substr(0,second);
    std::string origin_z_=origin_.substr(second+2,origin_.length()-second-3);
    float origin_x=atof(origin_x_.c_str());
    float origin_y=atof(origin_y_.c_str());
    float origin_z=atof(origin_z_.c_str());
    _origin[0] = origin_x;
    _origin[1] = origin_y;
    _origin[2] = origin_z;
}

void DistanceMap::computeMinMaxAlphaDist(MinMaxAlphaDist& MinMaxAlphaDist_,cv::Mat& img,
                                                int interestP_x,int interestP_y,float SearchWindow_radius)
{
    int imgheight=img.rows;
    int imgwidth=img.cols;

    float delta_theta=(float)_resolution/SearchWindow_radius;
    float R=(float)SearchWindow_radius/_resolution;
    float delta_length=1.0;

    float theta=0;

    float circle_theta=6.2831852;
    float avg_dist=0;
    float variance=0;
    int count=0;

    while(theta<circle_theta)
    {
        float length=0;
        while(length<R)
        {
            int col=(int)(cos(theta)*length)+MinMaxAlphaDist_.col;
            int row=(int)(sin(theta)*length)+MinMaxAlphaDist_.row;

            if(col>-1&&row>-1&&col<imgwidth&&row<imgheight)
            {

                if((int)img.at<uchar>(row,col)>5&&(int)img.at<uchar>(row,col)<250)
                {
                    break;
                }
                if((int)img.at<uchar>(row,col)<5)
                {

                    float templength=(float)_resolution*length;
                    //cout<<"templength="<<templength<<endl;
                    if(templength<MinMaxAlphaDist_.mindist)MinMaxAlphaDist_.mindist=templength;
                    if(templength>MinMaxAlphaDist_.maxdist)MinMaxAlphaDist_.maxdist=templength;

                    avg_dist+=templength;
                    count+=1;

                    break;
                }
            }

            length+=delta_length;

        }

        theta+=delta_theta;
    }

    avg_dist=(float)avg_dist/count;
    MinMaxAlphaDist_.avgdist=avg_dist;

    theta=0;
    //compute variance
    while(theta<circle_theta)
    {
        float length=0;
        while(length<R)
        {
            int col=(int)(cos(theta)*length)+MinMaxAlphaDist_.col;
            int row=(int)(sin(theta)*length)+MinMaxAlphaDist_.row;


            if(col>-1&&row>-1&&col<imgwidth&&row<imgheight)
            {

                if((int)img.at<uchar>(row,col)>5&&(int)img.at<uchar>(row,col)<250) //modified at 2017 5 28
                {
                    break;
                }

                if((int)img.at<uchar>(row,col)<5)
                {
                    float templength=(float)_resolution*length;
                    variance+=(templength-avg_dist)*(templength-avg_dist);
                    break;
                }
            }
            length+=delta_length;
        }
        theta+=delta_theta;
    }

    variance=(float)variance/count;
    MinMaxAlphaDist_.variance=variance;
}

bool DistanceMap::computeMinMaxAlphaDistforMap()
{
  const float search_radius = _max_scan_len;
    for(int col=0;col<_img.cols;col++)
    {
        for(int row=0;row<_img.rows;row++)
        {
            if((int)_img.at<uchar>(row,col)>250)
            {
                MinMaxAlphaDist MinMaxAlphaDist_;

                MinMaxAlphaDist_.col=col;
                MinMaxAlphaDist_.row=row;
                MinMaxAlphaDist_.x=_resolution*col;
                MinMaxAlphaDist_.y=_resolution*row;

                computeMinMaxAlphaDist(MinMaxAlphaDist_,_img,col,row,search_radius);

                if(MinMaxAlphaDist_.maxdist!=-1&&MinMaxAlphaDist_.mindist!=9999&&MinMaxAlphaDist_.avgdist!=-1&&MinMaxAlphaDist_.variance!=-1)
                {
                    //cout<<". "<<endl;
                    _vec_min_max_dist.push_back(MinMaxAlphaDist_);
                }
            }
        }
    }
    return writeDistMap();
}

bool DistanceMap::computeHistgramForMap(){
  const float search_radius = _max_scan_len;
  for(int col=0;col<_img.cols;col++){
      for(int row=0;row<_img.rows;row++){
          if((int)_img.at<uchar>(row, col)>250){
            int valid_count = 0;
              Histgram hist = computeHistAtPixel(_img, col, row, search_radius, valid_count);
              if(valid_count>0) _histgrams.push_back(hist);
          }
      }
  }
  return writeHistMap();
}

Histgram DistanceMap::computeHistAtPixel(const cv::Mat &img,
                                  const int x, const int y,
                                  const float search_radius, int& valid_count){
  Histgram hist(x, y, _hist_size);
  const float bin_step = (_max_scan_len - _min_scan_len) * 1.0 / _hist_size;
  int index = -1;
//  const float delta_theta = (float)_resolution/search_radius;
  const float delta_theta = 3.14159 / 180;
  const float radius_in_pixel = (float)search_radius/_resolution;
  const float delta_r = 1.0;

  float theta=0;
  float circle_theta=6.2831852;
  float scan_len = 0.;

  valid_count = 0;
  while(theta<circle_theta) { //theta
      float r=0;
      while(r<radius_in_pixel){ //radius
          int col=(int)(cos(theta)*r)+x;
          int row=(int)(sin(theta)*r)+y;

          if(col > -1 && row > -1 && col < img.cols && row < img.rows) {
              if((int)img.at<uchar>(row,col)>10 && (int)img.at<uchar>(row,col)<240){
                  break; //border which does not covered by map.
              }
              if((int)img.at<uchar>(row,col)<10){  //obstacles
                  scan_len=(float)_resolution*r;
                  if(scan_len<_min_scan_len || scan_len > _max_scan_len){
                  }else{
                    index = (scan_len-_min_scan_len) / bin_step;
                    hist._hist[index]++;
                    valid_count++;
                  }
                  break;
              }
          }
          r+=delta_r;
      }
      theta+=delta_theta;
  }
  return hist;
}


void BubbleSort(float* pData, int* idx, int count)
{
    float temp;
    int tempidx;
    for (int i = 1; i < count; i++)
    {
        for (int j = count - 1; j >= i; j--)
        {
            if (pData[j] < pData[j - 1])
            {
                temp = pData[j - 1];
                pData[j - 1] = pData[j];
                pData[j] = temp;

                tempidx=idx[j-1];
                idx[j-1]=idx[j];
                idx[j]=tempidx;
            }
        }
    }
}

bool DistanceMap::writeDistMap()
{
    std::cout << "Creating dispmaps ..." << std::endl;
    int count=_vec_min_max_dist.size();

    std::ofstream outputDistMap;
    outputDistMap.open(_map_dir + "/DistMaps.txt");
    for (int idx=0;idx<count;idx++)
    {
        outputDistMap << "idx=" <<idx<< std::endl;
        outputDistMap << "x="<<_vec_min_max_dist[idx].x<< std::endl;
        outputDistMap << "y="<<_vec_min_max_dist[idx].y<< std::endl;
        outputDistMap << "col="<<_vec_min_max_dist[idx].col<< std::endl;
        outputDistMap << "row="<<_vec_min_max_dist[idx].row<< std::endl;
        outputDistMap << "mindist="<<_vec_min_max_dist[idx].mindist<< std::endl;
        outputDistMap << "maxdist="<<_vec_min_max_dist[idx].maxdist<< std::endl;

        outputDistMap << "avgdist="<<_vec_min_max_dist[idx].avgdist<<std::endl;
        outputDistMap << "variance="<<_vec_min_max_dist[idx].variance<<std::endl;
    }

    //sort
    float MinDistList[count];
    std::vector<float> MinDistListVector;
    float MaxDistList[count];
    int IdxOfMinDistList[count];
    std::vector<int> IdxOfMinDistListVector;
    int IdxOfMaxDistList[count];

    for(int i=0; i<count;i++)
    {
        MinDistList[i]=_vec_min_max_dist[i].mindist;
        MaxDistList[i]=_vec_min_max_dist[i].maxdist;
        IdxOfMinDistList[i]=i;
        IdxOfMaxDistList[i]=i;

        MinDistListVector.push_back(_vec_min_max_dist[i].mindist);
        IdxOfMinDistListVector.push_back(i);
    }

    BubbleSort(MinDistList,IdxOfMinDistList, count);
    //BubbleSort(MaxDistList,IdxOfMaxDistList, count);

    //write
    std::ofstream outputIdxOfMinDist;
    outputIdxOfMinDist.open(_map_dir + "/IdxOfMinDist.txt");
    outputIdxOfMinDist << _origin[0] << " " << _origin[1] << " " << _img.rows*_resolution << std::endl;
    for (int i=0;i<count;i++)
    {
        outputIdxOfMinDist << "IdxOfMinDist=" <<IdxOfMinDistList[i]<< std::endl;
    }
    return true;
}

bool DistanceMap::writeHistMap(){
  std::ofstream ofs(_map_dir+"/"+_hist_file_name);
  if(!ofs){
    LOG(INFO)<<"open hist output file failed!";
    return false;
  }
  for(int i  = 0; i < _histgrams.size(); i++){
    ofs<<_histgrams[i]._row<<","<<_histgrams[i]._col<<",";
    auto h =_histgrams[i]._hist;
    for(int j = 0; j < h.size();j++){
      ofs<<h[j]<<",";
    }
    ofs<<"\n";
  }
  ofs.close();
  return true;
}
