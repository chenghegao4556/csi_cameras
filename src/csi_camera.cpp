//
// Created by chenghe on 12/13/19.
//

#include <csi_camera.h>

//////////////////////////////////////////////////////////////////////////////
CsiCameraPublisher::
CsiCameraPublisher(const int sensor_id,   const int frame_rate,
                   const int image_width, const int image_height):
                   gsconfig_(""),
                   pipeline_(NULL),
                   sink_(NULL),
                   sensor_id_(sensor_id),
                   frame_rate_(frame_rate),
                   image_width_(image_width),
                   image_height_(image_height)
{

}

//////////////////////////////////////////////////////////////////////////////
CsiCameraPublisher::
~CsiCameraPublisher()
{

}

bool CsiCameraPublisher::
Configure()
{

    gsconfig_ = CreatGstreamerPipeline();
    std::cout << "Using pipeline: \n\t" << gsconfig_ << "\n";

    std::stringstream ss, ss1;
    ss << "/csi_cam_" << sensor_id_ << "/image_raw";
    publisher_ = nh_.advertise<sensor_msgs::Image>(ss.str(), 1);

    ss1 << "/camera_" << sensor_id_ << "_frame";
    frame_id_ = ss1.str();

    return true;
}

//////////////////////////////////////////////////////////////////////////////
bool  CsiCameraPublisher::
InitStream()
{
    if(!gst_is_initialized())
    {
        // Initialize gstreamer pipeline
        ROS_DEBUG_STREAM( "Initializing gstreamer..." );
        gst_init(0,0);
    }

    GError *error = 0; // Assignment to zero is a gst requirement

    pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
    if (pipeline_ == NULL)
    {
        ROS_FATAL_STREAM( error->message );
        return false;
    }

    // Create RGB sink
    sink_ = gst_element_factory_make("appsink",NULL);
    GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));
    caps = gst_caps_new_simple( "video/x-raw",
                                "format", G_TYPE_STRING, "RGB",
                                NULL);

    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    // Set whether the sink should sync
    // Sometimes setting this to true can cause a large number of frames to be
    // dropped
    gst_base_sink_set_sync(
            GST_BASE_SINK(sink_),
            (sync_sink_) ? TRUE : FALSE);

    if(GST_IS_PIPELINE(pipeline_))
    {
        GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
        g_assert(outpad);

        GstElement *outelement = gst_pad_get_parent_element(outpad);
        g_assert(outelement);
        gst_object_unref(outpad);

        if(!gst_bin_add(GST_BIN(pipeline_), sink_))
        {
            ROS_FATAL("gst_bin_add() failed");
            gst_object_unref(outelement);
            gst_object_unref(pipeline_);
            return false;
        }

        if(!gst_element_link(outelement, sink_))
        {
            ROS_FATAL("GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
            gst_object_unref(outelement);
            gst_object_unref(pipeline_);
            return false;
        }

        gst_object_unref(outelement);
    }
    else
    {
        GstElement* launchpipe = pipeline_;
        pipeline_ = gst_pipeline_new(NULL);
        g_assert(pipeline_);

        gst_object_unparent(GST_OBJECT(launchpipe));

        gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

        if(!gst_element_link(launchpipe, sink_))
        {
            ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
            gst_object_unref(pipeline_);
            return false;
        }
    }

    // Calibration between ros::Time and gst timestamps
    GstClock * clock = gst_system_clock_obtain();
    ros::Time now = ros::Time::now();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
    ROS_INFO("Time offset: %.3f",time_offset_);

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
    {
        ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
        return false;
    }
    else
    {
        ROS_DEBUG_STREAM("Stream is PAUSED.");
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////
void CsiCameraPublisher::
Run()
{
    while(ros::ok()) {
        if(!this->Configure()) {
            ROS_FATAL("Failed to configure gscam!");
            continue;
        }

        if(!this->InitStream()) {
            ROS_FATAL("Failed to initialize gscam stream!");
            continue;
        }

        // Block while publishing
        this->PublishStream();

        this->CleanStream();

        ROS_INFO("GStreamer stream stopped!");

        //break;
    }
}

//////////////////////////////////////////////////////////////////////////////
void CsiCameraPublisher::
PublishStream()
{
    ROS_INFO_STREAM("Publishing stream...");
    if(gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        ROS_ERROR("Could not start stream!");
        return;
    }

    while(ros::ok())
    {

        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
        if(!sample)
        {
            ROS_ERROR("Could not get gstreamer sample.");
            break;
        }
        GstBuffer* buf = gst_sample_get_buffer(sample);
        GstMemory* memory = gst_buffer_get_memory(buf, 0);
        GstMapInfo info;

        gst_memory_map(memory, &info, GST_MAP_READ);
        gsize &buf_size = info.size;
        guint8* &buf_data = info.data;
        GstClockTime bt = gst_element_get_base_time(pipeline_);
        // Stop on end of stream
        if (!buf)
        {
            ROS_INFO("Stream ended.");
            break;
        }

        // Get the image width and height
        GstPad* pad = gst_element_get_static_pad(sink_, "sink");
        const GstCaps *caps = gst_pad_get_current_caps(pad);

        GstStructure *structure = gst_caps_get_structure(caps,0);
        gst_structure_get_int(structure,"width",&width_);
        gst_structure_get_int(structure,"height",&height_);
        const unsigned int expected_frame_size = width_ * height_ * 3;
        if (buf_size < expected_frame_size)
        {
            ROS_WARN_STREAM( "GStreamer image buffer underflow: Expected frame to be "
                                     << expected_frame_size << " bytes but got only "
                                     << (buf_size) << " bytes. (make sure frames are correctly encoded)");
        }

        // Construct Image message
        sensor_msgs::ImagePtr img(new sensor_msgs::Image());
        img->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->pts+bt)/1e6+time_offset_);
        img->header.frame_id = frame_id_;
        img->width = width_;
        img->height = height_;
        img->encoding = sensor_msgs::image_encodings::RGB8;
        img->is_bigendian = false;
        img->data.resize(expected_frame_size);
        img->step = width_ * 3;
        std::copy( buf_data, (buf_data)+(buf_size), img->data.begin());

        // Publish the image
        publisher_.publish(img);

        // Release the buffer
        if(buf)
        {
            gst_memory_unmap(memory, &info);
            gst_memory_unref(memory);
            gst_buffer_unref(buf);
        }
        ros::spinOnce();
    }
}

//////////////////////////////////////////////////////////////////////////////
void CsiCameraPublisher::
CleanStream()
{
    ROS_INFO("Stopping gstreamer pipeline...");
    if(pipeline_)
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////////
std::string CsiCameraPublisher::
CreatGstreamerPipeline() const
{

   return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id_) + 
           " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(image_width_) + 
           ", height=(int)" + std::to_string(image_height_) + 
		   ", format=(string)NV12, framerate=(fraction)" + std::to_string(frame_rate_) + 
		   "/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert";
}

