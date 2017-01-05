#include <ros/ros.h>
#include <thread>
#include <pg_spinnaker_camera/stereo_camera.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stereo_camera_node");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  pg_spinnaker_camera::StereoCamera sc(nh,nhp);
  sc.run();

  // ROS spin
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  spinner.spin(); // spin() will not return until the node has been shut down

  sc.stop();
  return 0;
}
