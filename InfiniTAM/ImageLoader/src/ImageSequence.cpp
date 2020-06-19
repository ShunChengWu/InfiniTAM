#ifdef WITH_OPENCV
#include "../CPU/ImageSequence.hpp"
#include <ImageLoader/ImageLoader.hpp>
#include <utility>
#include <dirent.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
//#include "../../bin/include/CxxTools/PathTool.hpp"
#define THROW(x, ...)  do { \
char buffer[1024];\
sprintf(buffer, x,  ##__VA_ARGS__); \
throw std::runtime_error(buffer);} while (0);


using namespace SCSLAM::IO;
namespace {
    std::string CheckEnd(std::string path){
        if (strcmp(&path[path.length() - 1], "/") != 0) {
            return path + "/";
        } else {
            return path;
        }
        return path;
    };
    std::vector<std::string> get_files_in_folder(std::string path, std::string type, bool return_full, bool sort) {
        std::vector<std::string> file_vec;
        DIR *dir;
        struct dirent *ent;

        if ((dir = opendir(path.c_str())) != NULL) {
            path = CheckEnd(path);
            while ((ent = readdir(dir)) != NULL) {
                if (ent->d_name[0] != '.') {
                    /* print all the files and directories within directory */
                    //printf ("%s\n", ent->d_name);
                    file_vec.push_back(return_full ? path + ent->d_name : ent->d_name);
                }
            }
            closedir(dir);
        } else {
            /* could not open directory */
            char buffer[1024];
            sprintf(buffer,"[%s][%s][%d] Unable to open directory at %s\n", __FILE__, __FUNCTION__,__LINE__, path.c_str());
            throw std::runtime_error(buffer);
        }
        if (sort) std::sort(file_vec.begin(), file_vec.end());

        if (type == "") return file_vec;

        std::vector<std::string> filtered;
        for (auto name: file_vec) {
            if (name.size() > type.size()) {
                std::string tmp = name.substr(name.size() - type.size(), type.size());
                if (tmp == type) filtered.push_back(name);
            }
        }
        return filtered;
    }
    std::vector<std::string> splitLine(std::string s , char delimiter) {
        std::vector<std::string>tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter))
        {
            if(token != "")
                tokens.push_back(token);
        }
        return tokens;
    }
    std::string removeType(const std::string &name) {
        auto t = name.find_last_of('.');
        if(t != std::string::npos)
            return name.substr(0, t);
        else
            return name;
    }
}

ImageSequence::ImageSequence(std::string path_intrincs, std::string path_extrinsics,
                             std::string path_depthImages, std::string path_colorImages)
        : img_counter(0), img_max_counter(-1), path_K_(std::move(path_intrincs)), path_extrin_(std::move(path_extrinsics)),
          path_depth_imgs_(std::move(path_depthImages)), path_color_imgs_(std::move(path_colorImages))
          {
              scale_ = 1.f/1000.f;
          }

int ImageSequence::Init() {
    if(!path_extrin_.empty())
        cam_extrinsics = get_files_in_folder(path_extrin_, "", true, true);
    else
        printf("[%s][%s] Warning. No ground truth pose file(s) are loaded.\n", __FILE__, __FUNCTION__);
    depth_images =
            get_files_in_folder(path_depth_imgs_, "", true, true);
    color_images =
            get_files_in_folder(path_color_imgs_, "", true, true);

    if(depth_images.size() != color_images.size())
        throw std::runtime_error("The size of depth images and color images mismatch!.\n");
//    std::vector<float> cam_K_vec =
//            ::IO::LoadMatrixFromFile(path_K_, 3, 3);
    // load Camera parameters
    loadIntrinsics(path_K_);

    //TODO: memory issue due to un-initialized color parameters
    cv::Mat depth_mat = cv::imread(depth_images[0], -1);
    cv::Mat color_mat = cv::imread(color_images[0], -1);


    //TODO: check the size of them maybe
    img_max_counter = depth_images.size();
    paramDepth.imgSize = {depth_mat.cols, depth_mat.rows};
    paramColor.imgSize = {color_mat.cols, color_mat.rows};
    return 1;
}

int ImageSequence::Next() {
    if (img_counter >= img_max_counter) return -1;
    return img_counter++;
}

int ImageSequence::NumberOfImages() {
    return img_max_counter;
}

int ImageSequence::getDepth(ORUtils::Image<float> *depthptr) {
    return getDepth(img_counter-1,depthptr);
}

int ImageSequence::getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}

int ImageSequence::getColor(ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    return getColor(img_counter-1,colorptr);
}

int ImageSequence::getPose(ORUtils::Matrix4<float> *pose) {
    return getPose(img_counter-1,pose);
}



int ImageSequence::getDepth(int idx, ORUtils::Image<float> *depthptr){
    if(depthptr == NULL) return -1;
    depth_mat = cv::imread(depth_images[idx], -1);
    if (depth_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;
    depthptr->ChangeDims(ORUtils::Vector2<int>(paramDepth.imgSize.width, paramDepth.imgSize.height));
    for (int r = 0; r < paramDepth.imgSize.height; ++r) {
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c] =
                    (float) (depth_mat.at<unsigned short>(r, c)) * scale_;
//            printf("%f ", depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c]);
        }
//        printf("\n");
    }
    return 1;
}

int ImageSequence::getDepth(int idx, ORUtils::Image<short> *depthptr){
    if(depthptr == NULL) return -1;
    depth_mat = cv::imread(depth_images[idx], -1);
    if (depth_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;
    depthptr->ChangeDims(ORUtils::Vector2<int>(paramDepth.imgSize.width, paramDepth.imgSize.height));
    for (int r = 0; r < paramDepth.imgSize.height; ++r) {
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c] =
                    (depth_mat.at<unsigned short>(r, c));
//            printf("%f ", depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c]);
        }
//        printf("\n");
    }
    return 1;
}

int ImageSequence::getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    if(colorptr == NULL) return -1;
    color_mat = cv::imread(color_images[idx], -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
    cv::flip(color_mat, color_mat, 0);
    //TODO: copy to our format
    colorptr->ChangeDims(ORUtils::Vector2<int>(paramColor.imgSize.width, paramColor.imgSize.height));
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].x =
                    (color_mat.at<uchar3>(r, c)).x;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].y =
                    (color_mat.at<uchar3>(r, c)).y;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].z =
                    (color_mat.at<uchar3>(r, c)).z;
        }
    return 1;
}

int ImageSequence::getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    if(colorptr == NULL) return -1;
    color_mat = cv::imread(color_images[idx], -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
    if(flipRGB_) cv::flip(color_mat, color_mat, 0);
    //TODO: copy to our format
    colorptr->ChangeDims(ORUtils::Vector2<int>(paramColor.imgSize.width, paramColor.imgSize.height));
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].x =
                    (color_mat.at<uchar3>(r, c)).x;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].y =
                    (color_mat.at<uchar3>(r, c)).y;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].z =
                    (color_mat.at<uchar3>(r, c)).z;
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].w = 255;
        }
    return 1;
}
#include <fstream>
#include <Eigen/Core>
inline Eigen::Map<Eigen::Matrix4f> getEigen4f(float *data){
    return Eigen::Map<Eigen::Matrix4f>(data);
}
int ImageSequence::getPose(int idx, ORUtils::Matrix4<float> *pose) {
    auto LoadMatrixFromFile = [](const std::string& filename, int M, int N)->std::vector<float>{
        std::vector<float> matrix;
        std::fstream fp (filename, std::ios::in);
        if(!fp)
            throw std::runtime_error("LoadMatrixFromFile::Cannot open file for reading.\n");

        float tmp;
        while (fp >> tmp) {
            matrix.push_back(tmp);
        }
        fp.close();
        if(matrix.size() == 4) {
            std::vector<float> tmp;
            tmp.push_back(matrix[0]);
            tmp.push_back(0);
            tmp.push_back(matrix[2]);
            tmp.push_back(0);
            tmp.push_back(matrix[1]);
            tmp.push_back(matrix[3]);
            tmp.push_back(0);
            tmp.push_back(0);
            tmp.push_back(1);
            matrix.swap(tmp);
        } else
        if(matrix.size() < (size_t)M*N){
            printf("Input format was not %d*%d matrix. matrix.size() = %zu\n", M,N,matrix.size());
            // The file is in different format.
            if(matrix.size() ==6 && M*N ==9){
                std::vector<float> tmp;
                tmp.push_back(matrix[2]);
                tmp.push_back(0);
                tmp.push_back(matrix[4]);
                tmp.push_back(0);
                tmp.push_back(matrix[3]);
                tmp.push_back(matrix[5]);
                tmp.push_back(0);
                tmp.push_back(0);
                tmp.push_back(1);
                matrix.swap(tmp);
            } else
                throw std::runtime_error("Input format doesn't support!.\n");

        }

        return matrix;
    };

    if(cam_extrinsics.empty()) return -1;
    std::vector<float> base2world_vec = LoadMatrixFromFile(cam_extrinsics[idx], 4, 4);
    base2world_vec[3] *= scale_;//TODO:scale not here
    base2world_vec[7] *= scale_;
    base2world_vec[11] *= scale_;
    memcpy(pose->m, base2world_vec.data(), sizeof(float) * 16);
    *pose = pose->t();

    auto pose_eigen = getEigen4f(pose->m);
    Eigen::Matrix4f transferMat = Eigen::Matrix4f::Identity();
    transferMat.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(-0.5 * EIGEN_PI, Eigen::Vector3f::UnitX()));
    pose_eigen =  transferMat * pose_eigen;
    return 1;
}
#endif