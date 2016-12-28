#include <pg_spinnaker_camera/spinnaker_camera.h>
#include <opencv2/opencv.hpp>

#include <csignal>
#include <thread>
#include <iostream>


volatile sig_atomic_t flag = 0;

void signal_handler(int sigal_number) {
  std::cout << sigal_number << std::endl;
  if (sigal_number == 2) flag = 1;
}

int main() {
  flag = 0;
  signal(SIGINT, signal_handler);
  SpinnakerCamera cam1("16401228"), cam2("16401229");
  cam1.PrintDeviceInfo();
  cam2.PrintDeviceInfo();

  cam1.SetImageMode(BAYER_RG8);
  cam2.SetImageMode(BAYER_RG8);

  cam1.DisableWhiteBalanceAuto();
  cam2.DisableWhiteBalanceAuto();
  cam1.SetWhiteBalanceBlue(850);
  cam1.SetWhiteBalanceRed(500);
  cam2.SetWhiteBalanceBlue(850);
  cam2.SetWhiteBalanceRed(500);

  cam1.DisableFrameRateAuto();
  cam2.DisableFrameRateAuto();
  cam1.SetFrameRate(1.0);
  cam2.SetFrameRate(1.0);

  cam1.DisableExposureAuto();
  cam2.DisableExposureAuto();
  cam1.SetExposureTime(5000);
  cam2.SetExposureTime(5000);

  cam1.DisableGainAuto();
  cam2.DisableGainAuto();
  cam1.SetGain(3.0);
  cam2.SetGain(3.0);

  // Set synchronization
//  cam1.DisableTrigger();
//  cam2.DisableTrigger();
//  cam1.EnableStrobe(LINE2);
//  cam2.EnableHardwareTrigger(LINE3);

  // Due to waterproof mounting, reverse camera
  cam1.SetReverseY();

  cam1.Start();
  cam2.Start();
  try {
    while (true) {
      cv::Mat img1 = cam1.GrabNextImage("bgr"), dst1;
      cv::Mat img2 = cam2.GrabNextImage("bgr"), dst2;
      cv::resize(img1, dst1, cv::Size(0, 0), 0.5, 0.5);
      cv::resize(img2, dst2, cv::Size(0, 0), 0.5, 0.5);
      std::vector<cv::Mat> channels1, channels2;
      cv::split(dst1, channels1);
      cv::split(dst2, channels2);
      std::vector<cv::Mat> channels3 = {channels1[0], channels1[1], channels2[2]};
      cv::Mat merged;
      cv::merge(channels3, merged);
      cv::imshow("cam1", dst1);
      cv::imshow("cam2", dst2);
      cv::imshow("merged", merged);
      cv::waitKey(1);
      if (flag) break;
    }
  } catch (Spinnaker::Exception &e) {
    std::cout << "Error: " << e.what() << std::endl;
  }
  cam1.End();
  cam2.End();
  return 0;
}