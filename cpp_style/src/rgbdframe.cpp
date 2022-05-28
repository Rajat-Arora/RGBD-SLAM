#include "rgbdframe.h"
#include "common_headers.h"
#include "parameter_reader.h"

using namespace rgbd_slam;

void FrameReader::init_tum(ParameterReader& para)
{
    dataset_dir = parameterReader.getData<string>("data_source");
    string associate_file  =  dataset_dir+"associate.txt";
    std::cout<<associate_file;
    ifstream fin(associate_file.c_str());
    if (!fin)
    {
        cerr<<"asscociate.txt not found"<<endl;

        cerr<<"run python associate.py rgb.txt depth.txt > associate.txt"<<endl;
        return;
    }

    while(!fin.eof())
    {
        string rgbTime, rgbFile, depthTime, depthFile;
        fin>>rgbTime>>rgbFile>>depthTime>>depthFile;
        if (!fin.good())
        {
            break;
        }
        rgbFiles.push_back(rgbFile);
        depthFiles.push_back(depthFile);
    }

    cout<<"Found total of "<<rgbFiles.size()<<" data record "<<endl;
    camera = parameterReader.getCamera();
    start_index = parameterReader.getData<int>("start_index");
    currentIndex = start_index;
}


RGBDFrame::Ptr FrameReader::next () 
{ 
    switch  (dataset_type) { 
    case  NYUD: 
        // TODO adds the interface of nyud 
        break ; 
    case  TUM: 
    { 
        if  (currentIndex < start_index || currentIndex >= rgbFiles. size ()) 
            return  nullptr ; 

        RGBDFrame:: Ptr    frame  ( new  RGBDFrame); 
        frame-> id  = currentIndex; 
        frame-> rgb  =  cv::imread ( dataset_dir + rgbFiles[currentIndex]); 
        frame-> depth  =  cv::imread ( dataset_dir + depthFiles[currentIndex], - 1 ); 
        std::cout<< dataset_dir + rgbFiles[currentIndex]<<std::endl;

        if  (frame-> rgb . data  ==  nullptr  || frame-> depth . data == nullptr ) 
        { 
            // data does not exist 
            return  nullptr ; 
        } 

        frame-> camera  =  this -> camera ; 
        currentIndex ++; 
        return  frame; 
    } 
    default : 
        break ; 
    } 

    return  nullptr ; 
} 
