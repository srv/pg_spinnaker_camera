#include <pg_spinnaker_camera/spinnaker_camera.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <thread>
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "binocular_demo");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  SpinnakerCamera cam1("16401228");
  // SpinnakerCamera cam2("16401229");

  if (cam1.IsConnected()) {
    cam1.PrintDeviceInfo();

    cam1.EnableStrobe(LINE2);
    cam1.EnableHardwareTrigger(LINE3);

    cam1.SetImageMode(BAYER_RG8);
    cam1.DisableWhiteBalanceAuto();
    cam1.SetWhiteBalanceBlue(1.0);
    cam1.SetWhiteBalanceRed(1.0);
    cam1.EnableFrameRateAuto();
    // cam1.SetFrameRate(1.0);
    cam1.DisableExposureAuto();
    cam1.SetExposureTime(5000);
    cam1.DisableGainAuto();
    cam1.SetGain(3.0);

    cam1.SetExposureEndEvent();

    std::cout << "Start" << std::endl;
    cam1.Start();
  }



  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
    r.sleep();
  std::cout << "End" << std::endl;
  cam1.End();
  ros::shutdown();

  return 0;

}