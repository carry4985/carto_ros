#include "distance_map.h"
#include <fstream>
DistanceMap::DistanceMap(std::string para_YamlFileRootPath)
    :m_YamlFileRootPath(para_YamlFileRootPath)
{
    std::ifstream fin( (m_YamlFileRootPath + "/map.yaml").c_str() );
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
        data[key] = value;

        if ( !fin.good() )
            break;
    }

    //resolution
    std::string resolution_=data["resolution"];
    resolution=atof(resolution_.c_str());

    std::string PgmFilePath = m_YamlFileRootPath + "/map.pgm";
    img=cv::imread(PgmFilePath, CV_LOAD_IMAGE_GRAYSCALE);

    //origin
    std::string origin_=data["origin"];
    int first=origin_.find(",");
    std::string origin_x_=origin_.substr(1,first-1);
    origin_=origin_.substr(first+2,origin_.length()-first-2);
    int second=origin_.find(",");
    std::string origin_y_=origin_.substr(0,second);
    std::string origin_z_=origin_.substr(second+2,origin_.length()-second-3);
    float origin_x=atof(origin_x_.c_str());
    float origin_y=atof(origin_y_.c_str());
    float origin_z=atof(origin_z_.c_str());
    origin[0] = origin_x;
    origin[1] = origin_y;
    origin[2] = origin_z;
}

void DistanceMap::ComputeMinMaxAlphaDist(MinMaxAlphaDist& MinMaxAlphaDist_,cv::Mat& img,
                                                int interestP_x,int interestP_y,float SearchWindow_radius)
{
    int imgheight=img.rows;
    int imgwidth=img.cols;

    float delta_theta=(float)resolution/SearchWindow_radius;
    float R=(float)SearchWindow_radius/resolution;
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

                    float templength=(float)resolution*length;
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
                    float templength=(float)resolution*length;
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

bool DistanceMap::ComputeMinMaxAlphaDistforimg(float SearchWindow_radius)
{
    for(int col=0;col<img.cols;col++)
    {
        for(int row=0;row<img.rows;row++)
        {
            if((int)img.at<uchar>(row,col)>250)
            {
                MinMaxAlphaDist MinMaxAlphaDist_;

                MinMaxAlphaDist_.col=col;
                MinMaxAlphaDist_.row=row;
                MinMaxAlphaDist_.x=resolution*col;
                MinMaxAlphaDist_.y=resolution*row;

                ComputeMinMaxAlphaDist(MinMaxAlphaDist_,img,col,row,SearchWindow_radius);

                if(MinMaxAlphaDist_.maxdist!=-1&&MinMaxAlphaDist_.mindist!=9999&&MinMaxAlphaDist_.avgdist!=-1&&MinMaxAlphaDist_.variance!=-1)
                {
                    //cout<<". "<<endl;
                    VectorMinMaxAlphaDist.push_back(MinMaxAlphaDist_);
                }
            }
        }
    }
    return WriteDistMap();
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

bool DistanceMap::WriteDistMap()
{
    std::cout << "Creating dispmaps ..." << std::endl;
    int count=VectorMinMaxAlphaDist.size();

    std::ofstream outputDistMap;
    outputDistMap.open(m_YamlFileRootPath + "/DistMaps.txt");
    for (int idx=0;idx<count;idx++)
    {
        outputDistMap << "idx=" <<idx<< std::endl;
        outputDistMap << "x="<<VectorMinMaxAlphaDist[idx].x<< std::endl;
        outputDistMap << "y="<<VectorMinMaxAlphaDist[idx].y<< std::endl;
        outputDistMap << "col="<<VectorMinMaxAlphaDist[idx].col<< std::endl;
        outputDistMap << "row="<<VectorMinMaxAlphaDist[idx].row<< std::endl;
        outputDistMap << "mindist="<<VectorMinMaxAlphaDist[idx].mindist<< std::endl;
        outputDistMap << "maxdist="<<VectorMinMaxAlphaDist[idx].maxdist<< std::endl;

        outputDistMap << "avgdist="<<VectorMinMaxAlphaDist[idx].avgdist<<std::endl;
        outputDistMap << "variance="<<VectorMinMaxAlphaDist[idx].variance<<std::endl;
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
        MinDistList[i]=VectorMinMaxAlphaDist[i].mindist;
        MaxDistList[i]=VectorMinMaxAlphaDist[i].maxdist;
        IdxOfMinDistList[i]=i;
        IdxOfMaxDistList[i]=i;

        MinDistListVector.push_back(VectorMinMaxAlphaDist[i].mindist);
        IdxOfMinDistListVector.push_back(i);
    }

    BubbleSort(MinDistList,IdxOfMinDistList, count);
    //BubbleSort(MaxDistList,IdxOfMaxDistList, count);

    //write
    std::ofstream outputIdxOfMinDist;
    outputIdxOfMinDist.open(m_YamlFileRootPath + "/IdxOfMinDist.txt");
    outputIdxOfMinDist << origin[0] << " " << origin[1] << " " << img.rows*resolution << std::endl;
    for (int i=0;i<count;i++)
    {
        outputIdxOfMinDist << "IdxOfMinDist=" <<IdxOfMinDistList[i]<< std::endl;
    }
    return true;
}
