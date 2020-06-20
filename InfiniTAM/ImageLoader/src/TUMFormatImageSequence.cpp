#include <ImageLoader/ImageLoader.hpp>
#include <dirent.h>
#include "../CPU/TUMFormatImageSequence.hpp"
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace SCSLAM::IO;
namespace {
    template<typename T, int x>
    Eigen::Map<Eigen::Matrix<T, x, x, Eigen::RowMajor>> getEigenRowMajor(T *data){
        return Eigen::Map<Eigen::Matrix<T, x, x, Eigen::RowMajor>>(data);
    }

    std::string find_parent_folder(std::string input, int times){
        if(input.empty()) return "";
        std::string output;
        if(input[input.size()-1] == '/') //input.pop_back();
            input = input.substr(0, input.length()-1);
        output = input;
        for (int i = 0; i < times; ++i){
            size_t temp = output.find_last_of("/");
            if (temp == std::string::npos){
                printf("[warning] Reach the most previous folder.");
                break;
            }
            output.assign(output, 0, output.find_last_of("/"));
        }
        return output;
    }
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
            perror("");
            exit(EXIT_FAILURE);
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
    std::string get_current_dir_name(std::string path){
        std::string output, tmp;
        tmp = find_parent_folder(path, 1);
        if(path[path.size()-1] == '/') //path.pop_back();
            path = path.substr(0, path.length()-1);
        output = path.substr(tmp.size()+1, path.size() - tmp.size());
        return output;
    }
    std::string getFileName(std::string pthIn){
        return removeType(get_current_dir_name(pthIn));
    }
}
inline std::vector<float> LoadMatrixFromFile(const std::string& filename, int M, int N) {
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
}

TUMFormatImageSequence::TUMFormatImageSequence(const std::string &path_to_folder,
        const std::string &path_to_intrinsic_file, const std::string &path_to_gt_file, bool verbal):
verbal_(verbal), img_counter(0), img_max_counter(0){
    scale_ = 1.f/5000.f;
    main_path = CheckEnd(path_to_folder);

    std::vector<float> cam_K_vec =LoadMatrixFromFile(path_to_intrinsic_file, 3, 3);
    paramDepth.SetFrom(cam_K_vec[0],cam_K_vec[4],cam_K_vec[2],cam_K_vec[5]);
    paramColor.SetFrom(cam_K_vec[0],cam_K_vec[4],cam_K_vec[2],cam_K_vec[5]);

    if(path_to_gt_file.empty()) {
        if(verbal) printf("[TUMFormatImageSequence] No path to ground truth trajectory provided.\n");
        return ;
    }
    std::fstream file (path_to_gt_file, std::ios::in);
    if(!file.is_open()) {
        printf("[Warning][TUMFormatImageSequence] Unable to load the provided ground truth trajectory file!\n");
        return;
    }

    std::string timestamp, tx,ty,tz,qx,qy,qz,qw, line;
    std::getline(file, line);
    while(std::getline(file, line, '\n')) {
        if(line[0] == '#') continue;

        std::vector<std::string> tokens = splitLine(line, ' ');
        if(tokens.size() != 8) {
            printf("[ERROR][ImageLoader] Couldn't read ground truth trajectory correctly on this line: %s\n",
                    line.c_str());
            exit(EXIT_FAILURE);
        }
//                for(size_t i=0;i<tokens.size();++i)
//                    printf("%s ", tokens[i].c_str());
//                printf("\n");



        Eigen::Quaternionf quat;
        Eigen::Vector3f t;
        t << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]);
        quat.x() = std::stod(tokens[4]);
        quat.y() = std::stod(tokens[5]);
        quat.z() = std::stod(tokens[6]);
        quat.w() = std::stod(tokens[7]);
//        quat.w() = std::stod(tokens[4]);
//        quat.x() = std::stod(tokens[5]);
//        quat.y() = std::stod(tokens[6]);
//        quat.z() = std::stod(tokens[7]);

//                std::cout << tokens[0] << " " << t.transpose() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << "\n";
        if(poses_.find(std::stod(tokens[0])) != poses_.end()) {
//            SCLOG(INFO) << "Dulpicate camera pose found!";
        }

        auto& pose = poses_[std::stod(tokens[0])];
        pose.reset(new ORUtils::Matrix4<float> ());
        auto eigenmat = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(pose->m);;
        eigenmat.setIdentity();
        eigenmat.topLeftCorner(3,3) = quat.toRotationMatrix();
        eigenmat.topRightCorner(3,1) = t;
    }
}
TUMFormatImageSequence::~TUMFormatImageSequence(){}
int TUMFormatImageSequence::Init(){
    bool initState = true;
    std::string path_rgb = main_path+"rgb";
    std::string path_depth = main_path+"depth";
    std::string path_association = main_path+"associations.txt";
    std::fstream file ( path_association, std::ios::in);
    if(!file.is_open()) {
        path_association = main_path+"associated.txt";
        file.open( path_association, std::ios::in);
    }
    if(!file.is_open()) {
        if(verbal_) printf("[ImageLoader] Couldn't find the given associations file! Try to create\n");
        auto color_names_raw = get_files_in_folder(path_rgb, "png", false, true);
        auto depth_names_raw = get_files_in_folder(path_depth, "png", false, true);

        // To float
        std::vector<double> color_names(color_names_raw.size());
        std::vector<double> depth_names(depth_names_raw.size());
        for (size_t i=0;i<color_names_raw.size(); ++i)
            color_names[i] = std::stod(color_names_raw[i]);
        for (size_t i=0;i<depth_names_raw.size(); ++i)
            depth_names[i] = std::stod(depth_names_raw[i]);
        std::sort(color_names.begin(),color_names.end());
        std::sort(depth_names.begin(), depth_names.end());

        file.open(path_association, std::ios::out);
        if (color_names.empty()) {
            printf("[ERROR][ImageLoader] Couldn't find a single image in the given path [%s]\n",
                   path_rgb.c_str());
            initState = false;
        }
        if (depth_names.empty()) {
            printf("[ERROR][ImageLoader] Couldn't find a single image in the given path [%s]\n",
                   path_depth.c_str());
            initState = false;
        }
        if (!initState) {
            exit(-1);
        }

        dataAssociation(depth_names, color_names);
        if(color_names.size() > depth_names.size())
            color_names.resize(depth_names.size());
        else
            depth_names.resize(color_names.size());

        for (size_t i=0; i < depth_names.size(); ++i){
            if(i>= color_names.size()) break;
            file << std::to_string(color_names[i]) << " " << "rgb/" + std::to_string(color_names[i]) + ".png" << " "
            << std::to_string(depth_names[i]) << " " << "depth/" + std::to_string(depth_names[i]) + ".png" << "\n";
//            color_images[color_names[i]] = main_path + "rgb/" + std::to_string(color_names[i]) + ".png";
//            depth_images[depth_names[i]] = main_path + "depth/" + std::to_string(depth_names[i]) + ".png";
            TUMTIMES tumtimes;
            tumtimes.t_rgb = color_names[i];
            tumtimes.s_rgb = main_path + "rgb/" + std::to_string(color_names[i]) + ".png";
            tumtimes.t_d   = depth_names[i];
            tumtimes.s_d   = main_path + "depth/" + std::to_string(depth_names[i]) + ".png";
            tum_time_map[i] = tumtimes;
        }
    } else {
        if(verbal_) printf("[ImageLoader] Found associations file!\n");
        std::string rgbn,rgbp,depthn,depthp;
        size_t counter=0;
        while(file  >> depthn >> depthp >> rgbn >>rgbp) {
            // First is rgb
//            printf("%f %f = %f\n", std::stod(depthn), std::stod(rgbn), std::abs(std::stod(depthn)-std::stod(rgbn)));
            TUMTIMES tumtimes;
            if(depthp.find("rgb") != std::string::npos) {
                tumtimes.t_rgb = std::stod(depthn);
                tumtimes.s_rgb = main_path + depthp;
                tumtimes.t_d   = std::stod(rgbn);
                tumtimes.s_d   = main_path + rgbp;
//                color_images[std::stod(depthn)] = (main_path + depthp);
//                depth_images[std::stod(rgbn)] = (main_path + rgbp);
            } else { // Second is rgb
                tumtimes.t_d   = std::stod(depthn);
                tumtimes.s_d   = main_path + depthp;
                tumtimes.t_rgb = std::stod(rgbn);
                tumtimes.s_rgb = main_path + rgbp;
//                depth_images[std::stod(depthn)] = (main_path + depthp);
//                color_images[std::stod(rgbn)] = (main_path + rgbp);
            }
            tum_time_map[counter++] = tumtimes;
        }
    }

    /// Filter Images with the time stamp of the provided ground truth trajectory
    if(!poses_.empty()) {
        poseAssociation();
    } else {
        size_t counter = 0;
//        for(auto &d : tum_time_map){
//            d.second.d_pose=0;
//        }
        for (const auto &d : tum_time_map){
            indicesMap_[counter].pose=0;
            indicesMap_[counter].depth=d.second.s_d;
            indicesMap_[counter].rgb = d.second.s_rgb;
            counter++;
        }
//        counter = 0;
//        for (auto &d : color_images) {
//            indicesMap_[counter].rgb = d.second;
//            counter++;
//        }
    }

    // check
    for(auto &t:indicesMap_){
        auto path = getFileName(t.second.depth);
//                printf("%f: %s\n", t.second.pose, path.c_str());
        double diff = std::abs(t.second.pose - std::stod(path));
        if(diff > 0.1) {
            printf("%f: %s = %f\n", t.second.pose, path.c_str(), diff);
        }
    }


    cv::Mat depth_mat = cv::imread(tum_time_map.begin()->second.s_d, -1);
    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;

    cv::Mat color_mat = cv::imread(tum_time_map.begin()->second.s_rgb, -1);
    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;

    img_max_counter = indicesMap_.size();
    return 1;
}
int TUMFormatImageSequence::Next(){
    if (img_counter >= img_max_counter) return -1;
    return img_counter++;
}
int TUMFormatImageSequence::NumberOfImages(){
    return (int) img_max_counter;
}

int TUMFormatImageSequence::getDepth(ORUtils::Image<float> *depthptr){
    return getDepth(img_counter-1,depthptr);
}
int TUMFormatImageSequence::getColor(ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr){
    return getColor(img_counter-1,colorptr);
}
int TUMFormatImageSequence::getPose(ORUtils::Matrix4<float> *pose){
    return getPose(img_counter-1,pose);
}


void TUMFormatImageSequence::dataAssociation(std::vector<double> &path1, std::vector<double> &path2,
        double maxOffset)
{
    std::sort(path1.begin(), path1.end());
    std::sort((path2.begin()), path2.end());
    auto iter_min = path2.end();
    auto iter = path2.begin();
    double min = maxOffset;
    std::vector<double> newp1, newp2;
    for (double & p1 : path1) {
        min = maxOffset;
        iter_min = path2.end();
        for(;iter != path2.end();++iter){
            double offset = (*iter - p1);
            if(offset < -maxOffset)continue;
            if(offset <= maxOffset){
                offset = std::abs(offset);
                if(offset < min) {
                    min = offset;
                    iter_min = iter;
                }
            }
            if(offset > maxOffset)break;
        }
        if(min == maxOffset || iter_min == path2.end()) continue;
        {
            newp1.push_back(p1);
            newp2.push_back(*iter_min);
            iter = iter_min;
        }

//                for (double & p2 : path2) {
//                    if (fabs(p1 - p2) < maxOffset) {
//                        newp1.push_back(p1);
//                        newp2.push_back(p2);
//                    }
//                }
    }

#ifndef NDEBUG
    for(size_t i=0;i<newp1.size();++i){
        double diff = std::abs(newp1[i] - newp2[i]);
        if(diff > maxOffset) {
            printf("%zu, %f: %f = %f\n", i, newp1[i],newp2[i], diff);
        }
    }
#endif

    path1.swap(newp1);
    path2.swap(newp2);
}

void TUMFormatImageSequence::poseAssociation(double maxOffset) {
    {
        size_t counter=0;
        double min;
        std::map<double, std::unique_ptr<ORUtils::Matrix4<float>>>::iterator iter_min;
        auto iter = poses_.begin();
        for(const auto &pair:tum_time_map){
            if(std::abs(pair.second.t_d - std::stod(getFileName(pair.second.s_d)))>maxOffset) continue;
            const double &t_depth = pair.second.t_d;
            min=maxOffset;
            iter_min = poses_.end();
            for(;iter != poses_.end(); ++iter) {
                const double &t_pose = iter->first;
                double offset = (t_pose - t_depth);
                if(offset < -maxOffset) continue;
                if(offset <= maxOffset) {
                    offset = std::abs(offset);
                    if(offset < min) {
                        min = offset;
                        iter_min = iter;
                    }
                }
                if(offset > maxOffset) break;
            }
            if(min == maxOffset || iter_min == poses_.end()) continue;
            {
                indicesMap_[counter].pose = iter_min->first;
                indicesMap_[counter].depth = pair.second.s_d;
                indicesMap_[counter].rgb = pair.second.s_rgb;
                counter++;
                iter = iter_min;
            }
        }
    }
}

int TUMFormatImageSequence::getDepth(int idx, ORUtils::Image<float> *depthptr) {
    if(depthptr == NULL) return -1;
    depth_mat = cv::imread(indicesMap_[idx].depth, -1);
    if (depth_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    if (color_mat.type() != CV_32FC1){
        color_mat.convertTo(color_mat, CV_32FC1);
    }
//            printf("depth_mat type: %d, channels: %d\n", depth_mat.type(), depth_mat.channels());
//            cv::imshow("DEPTH", depth_mat);
//            cv::waitKey();

    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;
    depthptr->ChangeDims(ORUtils::Vector2<int>(paramDepth.imgSize.width, paramDepth.imgSize.height));
    for (int r = 0; r < paramDepth.imgSize.height; ++r)
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c] =
                    (float) (depth_mat.at<unsigned short>(r, c)) * scale_;//TODO: change to write scale, using variable
        }
    return 1;
}

int TUMFormatImageSequence::getDepth(int idx, ORUtils::Image<short> *depthptr) {
    if(depthptr == NULL) return -1;
    depth_mat = cv::imread(indicesMap_[idx].depth, -1);
//    printf("depth: %s\n",indicesMap_[idx].depth.c_str());
    if (depth_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    if (color_mat.type() !=  CV_16UC1){
        color_mat.convertTo(color_mat, CV_16UC1);
    }
//            printf("depth_mat type: %d, channels: %d\n", depth_mat.type(), depth_mat.channels());
//            cv::imshow("DEPTH", depth_mat);
//            cv::waitKey();

    paramDepth.imgSize.height = depth_mat.rows;
    paramDepth.imgSize.width = depth_mat.cols;
    depthptr->ChangeDims(ORUtils::Vector2<int>(paramDepth.imgSize.width, paramDepth.imgSize.height));
    for (int r = 0; r < paramDepth.imgSize.height; ++r)
        for (int c = 0; c < paramDepth.imgSize.width; ++c) {
            depthptr->GetData(MEMORYDEVICE_CPU)[r * paramDepth.imgSize.width + c] =
                    (depth_mat.at<short>(r, c));//TODO: change to write scale, using variable
        }
    return 1;
}

int TUMFormatImageSequence::getColor(int idx, ORUtils::Image<ORUtils::Vector3<uchar>> *colorptr) {
    if(colorptr == nullptr) return -1;
    color_mat = cv::imread(indicesMap_[idx].rgb, -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    if (color_mat.type() != CV_8UC3){
        color_mat.convertTo(color_mat, CV_8UC3);
    }
//            printf("color_mat type: %d, channels: %d\n", color_mat.type(), color_mat.channels());
//            cv::imshow("DEPTH", color_mat);
//            cv::waitKey();

    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
    cv::flip(color_mat, color_mat, 0);
    //TODO: copy to our format
    colorptr->ChangeDims(ORUtils::Vector2<int>(paramColor.imgSize.width, paramColor.imgSize.height));
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].x = color_mat.at<cv::Vec3b>(r, c)[0];
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].y = color_mat.at<cv::Vec3b>(r, c)[1];
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].z = color_mat.at<cv::Vec3b>(r, c)[2];
        }
    return 1;
}
int TUMFormatImageSequence::getColor(int idx, ORUtils::Image<ORUtils::Vector4<uchar>> *colorptr) {
    if(colorptr == nullptr) return -1;
//    printf("rgb: %s\n",indicesMap_[idx].rgb.c_str());
    color_mat = cv::imread(indicesMap_[idx].rgb, -1);
    if (color_mat.empty()) {
        std::cout << "Error: depth image file not read!" << std::endl;
        cv::waitKey(0);
    }
    if (color_mat.type() != CV_8UC3){
        color_mat.convertTo(color_mat, CV_8UC3);
    }
//            printf("color_mat type: %d, channels: %d\n", color_mat.type(), color_mat.channels());
//            cv::imshow("DEPTH", color_mat);
//            cv::waitKey();

    paramColor.imgSize.height = color_mat.rows;
    paramColor.imgSize.width = color_mat.cols;
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
//    cv::flip(color_mat, color_mat, 0);
    //TODO: copy to our format
    colorptr->ChangeDims(ORUtils::Vector2<int>(paramColor.imgSize.width, paramColor.imgSize.height));
    for (int r = 0; r < paramColor.imgSize.height; ++r)
        for (int c = 0; c < paramColor.imgSize.width; ++c) {
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].x = color_mat.at<cv::Vec3b>(r, c)[0];
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].y = color_mat.at<cv::Vec3b>(r, c)[1];
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].z = color_mat.at<cv::Vec3b>(r, c)[2];
            colorptr->GetData(MEMORYDEVICE_CPU)[r * paramColor.imgSize.width + c].w = 255;
        }
    return 1;
}
int TUMFormatImageSequence::getPose(int idx, ORUtils::Matrix4<float> *pose) {
    if(indicesMap_[idx].pose == 0) return -1;
    if(poses_.find( indicesMap_[idx].pose ) == poses_.end() )return -1;
    pose->setValues(poses_[indicesMap_[idx].pose]->m);
    if(1)
    {
        auto pose_eigen = getEigenRowMajor<float, 4>(pose->m);
        pose_eigen.transposeInPlace();
//        Eigen::Matrix4f mat4f;
//        mat4f.setIdentity();

//        Eigen::Matrix3f mat3f;
//        mat3f = Eigen::AngleAxisf(-0.5 * EIGEN_PI, Eigen::Vector3f::UnitX());
//        mat4f.topLeftCorner<3, 3>() = mat3f;
//        pose_eigen = mat4f * pose_eigen;
//        mat3f = Eigen::AngleAxisf(-0.5 * EIGEN_PI, Eigen::Vector3f::UnitY());
//        mat4f.topLeftCorner<3, 3>() = mat3f;
//        pose_eigen = mat4f * pose_eigen;
    }
    return 1;
}