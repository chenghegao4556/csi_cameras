//
// Created by chenghe on 12/13/19.
//

#ifndef CSI_CAMERAS_CSI_CAMERA_H
#define CSI_CAMERAS_CSI_CAMERA_H

extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class CsiCameraPublisher
{
public:
    ///! smart pointer
    typedef std::shared_ptr<CsiCameraPublisher> Ptr;
    typedef std::shared_ptr<const CsiCameraPublisher> ConstPtr;

    /**
     * @brief constructor
     * @param[in] sensor_id
     * @param[in] frame_rate
     * @param[in] image_width
     * @param[in] image_height
     */
    CsiCameraPublisher(const int sensor_id,   const int frame_rate = 10,
                       const int image_width = 1948, const int image_height = 1096);

    /**
     * @brief delete default constructor
     */
    CsiCameraPublisher() = delete;

    /**
     * @brief destructor
     */
    ~CsiCameraPublisher();
	
	/**
     * @brief main thread
     */
    void Run();


protected:

    /**
     * @brief get gscam configuration
     * @return config
     */
    bool Configure();

    /**
     * @brief init gstream pipeline
     * @return
     */
    bool InitStream();

    /**
     * @brief publish image raw
     */
    void PublishStream();

    /**
     * @brief clean pipeline
     */
    void CleanStream();

    /**
     * @brief creat gstreamer pipeline
     * @return
     */
    std::string CreatGstreamerPipeline() const;
private:

    ///! parameters for gstreamer
	// General gstreamer configuration
    std::string gsconfig_;

    // Gstreamer structures
    GstElement *pipeline_;
    GstElement *sink_;

    // Appsink configuration
    bool sync_sink_;
    double time_offset_;

    ///! parameters for creating gstreamer pipeline
    int sensor_id_;
    int frame_rate_;
    int image_width_;
    int image_height_;

    ///! parameters for gstreamer get image resolution
    int width_;
    int height_;

    ///! frame coordinate
    std::string frame_id_;

    ///! ros related
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
};

#endif //CSI_CAMERAS_CSI_CAMERA_H
