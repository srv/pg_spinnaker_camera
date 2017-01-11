#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include <thread>
#include <pg_spinnaker_camera/stereo_camera.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}


int main(int argc, char** argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "MyNode", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // Init
	ros::init(argc, argv, "stereo_camera_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // The camera driver
  pg_spinnaker_camera::StereoCamera sc(nh,nhp);
  std::thread stereo_thread(&pg_spinnaker_camera::StereoCamera::run, &sc);

  // ROS spin
  ros::Rate r(50);
  while (!g_request_shutdown) {
    ros::spinOnce();
    r.sleep();
  }

  // Stop cameras
  sc.stop();

  // Stop node
  ros::shutdown();

  return 0;
}
