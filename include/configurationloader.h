//
// Created by chenghe on 1/10/20.
//

#ifndef CONFIGURATIONLOADER_CONFIGURATIONLOADER_H
#define CONFIGURATIONLOADER_CONFIGURATIONLOADER_H

#include <opencv2/core/core.hpp>
#include <memory>

/**
 * @brief write a demo configuration file
 */
void WriteDemoConfigurationFile(const std::string& file_name);

class CameraParameter
{
public:
    ///! smart pointer
    typedef std::shared_ptr<CameraParameter> Ptr;
    typedef std::shared_ptr<const CameraParameter> ConstPtr;

    /**
     * @brief constructor
     * @param[in] camera_id
     * @param[in] intrinsic
     * @param[in] distortion_coefficient
     */
    CameraParameter(const int camera_id, const size_t height, const size_t width,
                    const cv::Mat& intrinsic, const cv::Mat& distortion_coefficient):
        camera_id_(camera_id),
        height_(height),
        width_(width),
        intrinsic_(intrinsic),
        distortion_coefficient_(distortion_coefficient)
    {

    }

    /**
     * @brief delete default constructor
     */
    CameraParameter() = delete;

    /**
     * @brief get camera id
     */
    size_t GetCameraId() const
    {
        return camera_id_;
    }

    /**
     * @brief get height
     */
    size_t GetHeight() const
    {
        return height_;
    }

    /**
     * @brief get width
     */
    size_t GetWidth() const
    {
        return width_;
    }

    /**
     * @brief get intrinsic matrix
     */
    const cv::Mat& GetIntrinsic() const
    {
        return intrinsic_;
    }

    /**
     * @brief get distortion parameters
     * @return
     */
    const cv::Mat& GetDistortionCoefficient() const
    {
        return distortion_coefficient_;
    }
private:

    size_t camera_id_;
    size_t height_;
    size_t width_;
    cv::Mat intrinsic_;
    cv::Mat distortion_coefficient_;
};

class ConfigurationLoader
{
public:
    ///! smart pointer
    typedef std::shared_ptr<ConfigurationLoader> Ptr;
    typedef std::shared_ptr<const ConfigurationLoader> ConstPtr;

    /**
     * @brief constructor
     * @param[in] file_name
     */
    ConfigurationLoader(const std::string& file_name);

    void Clear()
    {
        used_camera_ids_.clear();
        camera_parameters_.clear();
    }

    /**
     * @brief number of used cameras
     */
    size_t NumberOfUsedCameras() const
    {
        return used_camera_ids_.size();
    }

    /**
     * @brief total number of cameras
     */
    size_t NumberOfCameras() const
    {
        return camera_parameters_.size();
    }

    /**
     * @brief ids of used cameras
     */
    std::vector<size_t > UsedCameraId() const
    {
        return used_camera_ids_;
    }

    /**
     * @brief camera parameters of all camera
     */
    const std::vector<CameraParameter>& GetCameraParameters() const
    {
        return camera_parameters_;
    }

    /**
     * @brief get id_th camera parameters
     */
    const CameraParameter& GetCameraParameter(const size_t id) const
    {
        assert(id < camera_parameters_.size());

        return camera_parameters_[id];
    }

protected:
    bool Load(const std::string& file_name);

private:
    std::vector<size_t> used_camera_ids_;
    std::vector<CameraParameter> camera_parameters_;
};

#endif //CONFIGURATIONLOADER_CONFIGURATIONLOADER_H
