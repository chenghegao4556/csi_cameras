#include <csi_camera.h>

int main(int argc, char** argv)
{

    const int num_sensors = 6;
    ros::init(argc, argv, "image_publisher");
  	std::vector<CsiCameraPublisher> csi_cameras;
	std::vector<std::thread> threads;
  	for(int i = 0; i < num_sensors; ++i)
    {
  	    csi_cameras.emplace_back(CsiCameraPublisher(i));
    }
    for(int i = 0; i < num_sensors; ++i)
    {
		threads.emplace_back(std::bind(&CsiCameraPublisher::Run, &csi_cameras[i]));
    }

	for(int i = 0; i < num_sensors; ++i)
    {
        threads[i].join();
    }
    return 0;
}


