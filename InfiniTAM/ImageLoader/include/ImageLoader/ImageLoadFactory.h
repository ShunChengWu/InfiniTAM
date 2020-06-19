#pragma once
#include "../../CPU/TUMFormatImageSequence.hpp"

#ifdef WITH_OPENCV
#include "../../CPU/ScanNet.hpp"
#include "../../CPU/ImageSequence.hpp"
#endif

namespace SCSLAM {

    struct ImageLoaderFactory {
        static SCSLAM::IO::ImageLoader * MakeImageLoader(SCSLAM::IO::InputDateType::INPUTDATATYPE inputdatatype, const std::string &folder = "", const std::string &cam_K = "",
                                                         const std::string &gt = "", const std::string &depthFolderName = "", const std::string &colorFolderName = "") {
            SCSLAM::IO::ImageLoader *imageLoader = nullptr;
            switch (inputdatatype) {
                case SCSLAM::IO::InputDateType::INPUTTYPE_SCANNET: {
                    imageLoader = (new SCSLAM::IO::ScanNet(folder));
                }
                    break;
                case SCSLAM::IO::InputDateType::INPUTTYPE_TUM: {
                    imageLoader = (new SCSLAM::IO::TUMFormatImageSequence(folder,
                                                                          cam_K,
                                                                          gt));
                    imageLoader->setDepthScale(1 / 5000.f);
                    break;
                }
            }
            return imageLoader;
        }
    };

}