#include <csi_camera.h>
#include <configurationloader.h>
int main(int argc, char** argv)
{
	std::string file_name("/home/xavier/catkin_ws/src/csi_cameras/config/csi_camera_configure.json");
    ConfigurationLoader cl(file_name);
    std::cout<<"number of cameras"<<cl.NumberOfCameras()<<std::endl;
    std::cout<<"number of used cameras"<<cl.NumberOfUsedCameras()<<std::endl;
    for(size_t i = 0; i < cl.NumberOfCameras(); ++i)
    {
        const auto& camera = cl.GetCameraParameter(i);
        std::cout<<"camera id "<<camera.GetCameraId()<<std::endl;
        std::cout<<"camera height "<<camera.GetHeight()<<std::endl;
        std::cout<<"camera width "<<camera.GetWidth()<<std::endl;
        std::cout<<"camera intrinsic "<<std::endl;
        std::cout<<camera.GetIntrinsic()<<std::endl;
        std::cout<<"camera distortion "<<std::endl;
        std::cout<<camera.GetDistortionCoefficient()<<std::endl;
    }
    for(size_t i = 0; i < cl.NumberOfUsedCameras(); ++i)
    {
      	std::cout<<"used camera id "<<cl.UsedCameraId()[i]<<std::endl;
    }
    ros::init(argc, argv, "image_publisher");
  	std::vector<CsiCameraPublisher> csi_cameras;
	std::vector<std::thread> threads;
  	for(const auto& i: cl.UsedCameraId())
    {
  	    csi_cameras.emplace_back(CsiCameraPublisher(i));
    }
    for(size_t i = 0; i < cl.NumberOfUsedCameras(); ++i)
    {
		threads.emplace_back(std::bind(&CsiCameraPublisher::Run, &csi_cameras[i]));
    }

	for(size_t i = 0; i < cl.NumberOfUsedCameras(); ++i)
    {
        threads[i].join();
    }
    return 0;
}
