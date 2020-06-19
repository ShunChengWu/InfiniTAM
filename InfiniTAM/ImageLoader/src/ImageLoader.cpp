#include <ImageLoader/ImageLoader.hpp>
#include <fstream>

using namespace SCSLAM::IO;

int ImageLoader::getColorParams(int& width, int& height, float& fx, float& fy, float& cx, float& cy){
    width = paramColor.imgSize.width;
    height = paramColor.imgSize.height;
    fx = paramColor.projectionParamsSimple.fx;
    fy = paramColor.projectionParamsSimple.fy;
    cx = paramColor.projectionParamsSimple.px;
    cy = paramColor.projectionParamsSimple.py;
    return 1;
}

int ImageLoader::getDepthParams(int& width, int& height, float& fx, float& fy, float& cx, float& cy){
    width = paramDepth.imgSize.width;
    height = paramDepth.imgSize.height;
    fx = paramDepth.projectionParamsSimple.fx;
    fy = paramDepth.projectionParamsSimple.fy;
    cx = paramDepth.projectionParamsSimple.px;
    cy = paramDepth.projectionParamsSimple.py;
    return 1;
}

void ImageLoader::loadIntrinsics(const std::string &path)
{
    std::fstream file(path, std::ios::in);
    if(!file.is_open()) {
        char buffer[1024];
        sprintf(buffer,"[%s][%s][%d]Cannot open file for reading intrinsic parameter(%s)\n", __FILE__,__FUNCTION__,__LINE__,path.c_str());
        throw std::runtime_error(buffer);
    }
    auto getParams=[&](std::fstream &file){
        std::string token;
        file >> token;
        float fx,fy,cx,cy;
        file >> fx >> fy >> cx >> cy;
        if(token == "depth"){
            paramDepth.SetFrom(fx,fy,cx,cy);
        } else if (token == "color"){
            paramColor.SetFrom(fx,fy,cx,cy);
        }
    };
    getParams(file);
    getParams(file);
}