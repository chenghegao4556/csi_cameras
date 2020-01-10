//
// Created by chenghe on 1/10/20.
//
#include <configurationloader.h>
#include <iostream>
size_t StringToSizeT(const std::string &string)
{
    std::istringstream is(string);
    size_t i;
    is >> i;

    return i;
}

void WriteDemoConfigurationFile(const std::string& file_name)
{
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 960, 0, 960, 0, 960, 540, 0, 0, 1 );
    cv::Mat D = ( cv::Mat_<double> ( 1,4 ) << 0, 0, 0, 0);
    std::vector<size_t> used_cameras;
    size_t height = 1080;
    size_t width = 1920;
    for(size_t i = 0; i < 6; ++i)
    {
        used_cameras.push_back(i);
    }

    cv::FileStorage fs(file_name.c_str(), cv::FileStorage::WRITE);
    if(!fs.isOpened()) throw std::string("Could not open file ") + file_name;
    fs << "CsiCameras" << "{";
    fs << "Cameras" << "[";
    for(size_t i = 0; i < 6; ++i)
    {
        fs << "{:"
           << "ID" << std::to_string(i)
           << "Height" << std::to_string(height)
           << "Width"  << std::to_string(width)
           << "Intrinsic"  << K
           << "Distortion" << D
           << "}";
    }
    fs << "]"; // Cameras
    fs << "UsedCameras" << "[";
    for(size_t i = 0; i < 6; ++i)
    {
        fs << "{:"
           << "ID" << std::to_string(i)
           << "}";
    }
    fs << "]"; // Cameras
    fs << "}"; //CsiCameras
    fs.release();
}

ConfigurationLoader::
ConfigurationLoader(const std::string &file_name)
{
    Load(file_name);
}

bool ConfigurationLoader::
Load(const std::string &file_name)
{
    cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::string("Could not open file ") + file_name;
    Clear();

    cv::FileNode fdb = fs["CsiCameras"];
    cv::FileNode fn = fdb["Cameras"];
    for(size_t i = 0; i < fn.size(); ++i)
    {
        size_t id = StringToSizeT(fn[i]["ID"]);
        size_t height = StringToSizeT(fn[i]["Height"]);
        size_t width  = StringToSizeT(fn[i]["Width"]);
        cv::Mat intrinsic = fn[i]["Intrinsic"].mat();
        cv::Mat distortion = fn[i]["Distortion"].mat();
        camera_parameters_.emplace_back(id, height, width, intrinsic, distortion);
    }
    cv::FileNode fu = fdb["UsedCameras"];
    for(size_t i = 0; i < fu.size(); ++i)
    {
        size_t id = StringToSizeT(fu[i]["ID"]);
        used_camera_ids_.push_back(id);
    }
}
