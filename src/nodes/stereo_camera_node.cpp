#include <ros/ros.h>
#include <thread>
#include <pg_spinnaker_camera/stereo_camera.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stereo_camera_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  pg_spinnaker_camera::StereoCamera sc(nh,nhp);
  std::thread stereo_thread(&pg_spinnaker_camera::StereoCamera::run, &sc);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
    r.sleep();

  sc.stop();
  ros::shutdown();

  return 0;
}
