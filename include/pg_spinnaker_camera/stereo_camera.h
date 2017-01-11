/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include <pg_spinnaker_camera/spinnaker_camera.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>

namespace pg_spinnaker_camera {
class StereoCameraConfig {
 public:
  int width;
  int height;
  int offset_x;
  int offset_y;
  int binning_vertical;
  int decimation_vertical;
  std::string pixel_format;
  std::string pixel_size;
  std::string video_mode;
  std::string line_selector;
  std::string line_mode;
  std::string line_source;
  std::string trigger_source;
  std::string trigger_selector;
  std::string trigger_activation;
  std::string acquisition_frame_rate_auto;
  bool acquisition_frame_rate_enabled;
  float acquisition_frame_rate;
  std::string exposure_auto;
  float exposure_time;
  float auto_exposure_exposure_time_upper_limit;
  float gain;
  std::string gain_auto;
  float auto_gain_lower_limit;
  float auto_gain_upper_limit;
  std::string balance_white_auto;
  float balance_ratio_red;
  float balance_ratio_blue;
  float black_level;
  float gamma;
  float sharpness;
  float hue;
  float saturation;
};

class StereoCamera {
 public:
  StereoCamera(ros::NodeHandle nh, ros::NodeHandle nhp);
  void run();
  void stop();

 private:

  // Cameras
  std::shared_ptr<SpinnakerCamera> l_cam_;
  std::shared_ptr<SpinnakerCamera> r_cam_;

  // Parameters
  std::string left_serial_number_;
  std::string right_serial_number_;
  std::string left_camera_info_url_;
  std::string right_camera_info_url_;
  StereoCameraConfig config_;

  // Publish counters
  long int left_counter_;
  long int right_counter_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  // Image transport
  image_transport::ImageTransport it_;

  // ROS Camera publisher
  image_transport::CameraPublisher left_pub_;
  image_transport::CameraPublisher right_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_man_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> right_info_man_;

  // Sync
  std::vector<sensor_msgs::Image> r_imgs_buffer_;
  std::vector<sensor_msgs::Image> l_imgs_buffer_;
  int imgs_buffer_size_;
  std::mutex l_sync_mutex_;
  std::mutex r_sync_mutex_;
  double max_sec_diff_;

  // Stop handlers
  bool l_stop_;
  bool r_stop_;

  void leftFrameThread();
  void rightFrameThread();

  void configureCamera(const std::shared_ptr<SpinnakerCamera>& cam, bool is_left);
};
}
#endif
