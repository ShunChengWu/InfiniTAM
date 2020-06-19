#pragma once
#include "../../ORUtils/Image.h"
#include "../../ORUtils/Matrix.h"
#include <map>
#include <opencv2/core.hpp>
#include "../include/ImageLoader/ImageLoader.hpp"
namespace SCSLAM {
    namespace IO{
        class TUMFormatImageSequence : public ImageLoader {
        public:
            struct Names{
                double pose;
                std::string rgb, depth;
                Names():pose(0), rgb(""), depth(""){}
                Names(float p, std::string r, std::string d):pose(p),rgb(r),depth(d){}
            } ;
            /// Folder must contains: rgb and depth folder. Optional: associations.txt
            TUMFormatImageSequence(const std::string &path_to_folder, const std::string &path_to_intrinsic_file,
                    const std::string &path_to_gt_file = "",
                    bool verbal = false);
            ~TUMFormatImageSequence();
            int Init() override;
            int Next() override;
            int NumberOfImages() override;
            int getDepth(ORUtils::Image<float> *depthptr) override;
            int getColor(ORUtils::Image<ORUtils::Vector3<unsigned char>> *colorptr) override;
            int getDepth(ORUtils::Image<short> *depthptr) override  {return -1;};
            int getColor(ORUtils::Image<ORUtils::Vector4<unsigned char>> *colorptr) override  {return -1;};
            int getPose(ORUtils::Matrix4<float> *pose) override;
            int getLabel(ORUtils::Image<ushort> *labelptr) override {return -1;}

            int getDepth(int idx, ORUtils::Image<float> *depthptr) override ;
            int getDepth(int idx, ORUtils::Image<short> *depthptr) override ;
            int getColor(int idx, ORUtils::Image<ORUtils::Vector3<unsigned char>> *colorptr) override ;
            int getColor(int idx, ORUtils::Image<ORUtils::Vector4<unsigned char>> *colorptr) override ;
            int getLabel(int idx, ORUtils::Image<ushort> *labelptr) {return -1;}
            int getPose(int idx, ORUtils::Matrix4<float> *pose) override ;
        protected:
            std::map<size_t, Names> indicesMap_;
            std::map<double, std::unique_ptr<ORUtils::Matrix4<float>>> poses_;
//            std::map<double, std::string> depth_images;
//            std::map<double, std::string> color_images;
            struct TUMTIMES {
                double t_rgb, t_d;
                std::string s_rgb, s_d;
                double d_pose;
            };
            std::map<size_t,TUMTIMES> tum_time_map;

            std::string main_path;
            bool verbal_;
            size_t img_counter, img_max_counter;
            cv::Mat color_mat, depth_mat;
            /// For every path in path1, find all the possible candidate in path2 in maximum time offset of maxOffset (Unit: Meter)
            static void dataAssociation(std::vector<double> &path1, std::vector<double> &path2, double maxOffset = 0.02);
            /// for each pair, associate it with a closet ground truth pose within the given margin.
            void poseAssociation(double maxOffset = 0.02);
        };
    }
}
