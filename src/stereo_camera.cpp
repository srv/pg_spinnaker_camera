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

#include <pg_spinnaker_camera/stereo_camera.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace pg_spinnaker_camera {

StereoCamera::StereoCamera(ros::NodeHandle nh, ros::NodeHandle nhp)
: l_cam_(NULL), r_cam_(NULL), left_counter_(0), right_counter_(0), nh_(nh), nhp_(nhp), it_(nh) {

  // Get the parameters
  nhp_.param("left_serial_number", left_serial_number_, std::string("16401228"));
  nhp_.param("right_serial_number", right_serial_number_, std::string("16401229"));
  nhp_.param("left_camera_info_url", left_camera_info_url_, std::string(""));
  nhp_.param("right_camera_info_url", right_camera_info_url_, std::string(""));

  nhp_.getParam("width", config_.width);
  nhp_.getParam("height", config_.height);
  nhp_.getParam("offset_x", config_.offset_x);
  nhp_.getParam("offset_y", config_.offset_y);
  nhp_.getParam("binning_vertical", config_.binning_vertical);
  nhp_.getParam("decimation_vertical", config_.decimation_vertical);
  nhp_.getParam("pixel_format", config_.pixel_format);
  nhp_.getParam("pixel_coding", config_.pixel_coding);
  nhp_.getParam("video_mode", config_.video_mode);
  nhp_.getParam("line_selector", config_.line_selector);
  nhp_.getParam("line_mode", config_.line_mode);
  nhp_.getParam("line_source", config_.line_source);
  nhp_.getParam("trigger_source", config_.trigger_source);
  nhp_.getParam("trigger_selector", config_.trigger_selector);
  nhp_.getParam("trigger_activation", config_.trigger_activation);
  nhp_.getParam("acquisition_frame_rate_auto", config_.acquisition_frame_rate_auto);
  nhp_.getParam("acquisition_frame_rate_enabled", config_.acquisition_frame_rate_enabled);
  nhp_.getParam("acquisition_frame_rate", config_.acquisition_frame_rate);
  nhp_.getParam("exposure_auto", config_.exposure_auto);
  nhp_.getParam("exposure_time", config_.exposure_time);
  nhp_.getParam("auto_exposure_exposure_time_upper_limit", config_.auto_exposure_exposure_time_upper_limit);
  nhp_.getParam("gain", config_.gain);
  nhp_.getParam("gain_auto", config_.gain_auto);
  nhp_.getParam("auto_gain_lower_limit", config_.auto_gain_lower_limit);
  nhp_.getParam("auto_gain_upper_limit", config_.auto_gain_upper_limit);
  nhp_.getParam("black_level", config_.black_level);
  nhp_.getParam("black_level_auto", config_.black_level_auto);
  nhp_.getParam("gamma", config_.gamma);
  nhp_.getParam("gamma_enabled", config_.gamma_enabled);
  nhp_.getParam("sharpness", config_.sharpness);
  nhp_.getParam("sharpness_enabled", config_.sharpness_enabled);
  nhp_.getParam("sharpness_auto", config_.sharpness_auto);
  nhp_.getParam("hue", config_.hue);
  nhp_.getParam("hue_enabled", config_.hue_enabled);
  nhp_.getParam("hue_auto", config_.hue_auto);
  nhp_.getParam("saturation", config_.saturation);
  nhp_.getParam("saturation_enabled", config_.saturation_enabled);
  nhp_.getParam("saturation_auto", config_.saturation_auto);
  nhp_.getParam("balance_white_auto", config_.balance_white_auto);
  nhp_.getParam("balance_ratio_red", config_.balance_ratio_red);
  nhp_.getParam("balance_ratio_blue", config_.balance_ratio_blue);

  configureCamera(l_cam_, true);
  configureCamera(r_cam_, false);
}

StereoCamera::~StereoCamera() {
  if (l_cam_) {
    l_cam_->End();
  }
  if (r_cam_) {
    r_cam_->End();
  }
}

void StereoCamera::run() {
  // Set the image publishers before the streaming
  left_pub_  = it_.advertiseCamera("/stereo_forward/left/image_raw",  1);
  right_pub_ = it_.advertiseCamera("/stereo_forward/right/image_raw", 1);

  // Set camera info managers
  left_info_man_  = std::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp_, "left"),"left_optical", left_camera_info_url_));
  right_info_man_ = std::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp_, "right"),"right_optical", right_camera_info_url_));

  // Bind the callbacks with the cameras
  //std::function<void(pg_spinnaker_camera::StereoCamera&)> leftFrameThreadCaller = &pg_spinnaker_camera::StereoCamera::leftFrameThread;
  //std::function<void(pg_spinnaker_camera::StereoCamera&)> rightFrameThreadCaller = &pg_spinnaker_camera::StereoCamera::rightFrameThread;
  l_cam_->setCallback(std::bind(&pg_spinnaker_camera::StereoCamera::leftFrameThread, this));
  r_cam_->setCallback(std::bind(&pg_spinnaker_camera::StereoCamera::rightFrameThread, this));
}

void StereoCamera::leftFrameThread() {
  cv::Mat left_img = l_cam_->GrabNextImage();

  if (left_img.rows > 0) {
    // Setup header
    std_msgs::Header header;
    header.seq = left_counter_;
    header.stamp = ros::Time::now();  //l_cam_->GetImageTimestamp();

    // Setup image
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BAYER_GBRG8, left_img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

    // Camera info
    sensor_msgs::CameraInfo ci = left_info_man_->getCameraInfo();
    ci.header.stamp = ros::Time::now();  //l_cam_->GetImageTimestamp();

    // Set frame_id
    img_msg.header.frame_id = ci.header.frame_id;
    // Publish
    left_pub_.publish(img_msg, ci);
    left_counter_++;
  }
}

void StereoCamera::rightFrameThread() {
  cv::Mat right_img = r_cam_->GrabNextImage();

  if (right_img.rows > 0) {
    // Setup header
    std_msgs::Header header;
    header.seq = right_counter_;
    header.stamp = ros::Time::now();  //r_cam_->GetImageTimestamp();

    // Setup image
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BAYER_GBRG8, right_img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

    // Camera info
    sensor_msgs::CameraInfo ci = right_info_man_->getCameraInfo();
    ci.header.stamp = ros::Time::now();  //r_cam_->GetImageTimestamp();

    // Set frame_id
    img_msg.header.frame_id = ci.header.frame_id;
    // Publish
    right_pub_.publish(img_msg, ci);
    right_counter_++;
  }
}

void StereoCamera::configureCamera(const std::shared_ptr<SpinnakerCamera>& cam,
                                   bool is_left) {
  // Image format
  cam->set("Width", config_.width);
  cam->set("Height", config_.height);
  cam->set("OffsetX", config_.offset_x);
  cam->set("OffsetY", config_.offset_y);
  cam->set("BinningVertical", config_.binning_vertical);
  cam->set("DecimationVertical", config_.decimation_vertical);
  cam->set("PixelFormat", config_.pixel_format);
  cam->set("PixelCoding", config_.pixel_coding);
  cam->set("VideoMode", config_.video_mode);

  if(is_left) {
    // Set strobe (LEFT CAMERA ONLY)
    cam->set("LineSelector", config_.line_selector);
    cam->set("LineMode", config_.line_mode);
    cam->set("LineSource", config_.line_source);
  } else {
    // Trigger (Must be switched off to change source!)
    // (RIGHT CAMERA ONLY)
    cam->set("TriggerMode", std::string("Off"));
    cam->set("TriggerSource", config_.trigger_source);
    cam->set("TriggerMode", std::string("On"));
    cam->set("TriggerSelector", config_.trigger_selector);
    cam->set("TriggerActivation", config_.trigger_activation);
  }

  cam->set("AcquisitionFrameRateAuto", config_.acquisition_frame_rate_auto);
  cam->set("AcquisitionFrameRateEnabled", config_.acquisition_frame_rate_enabled);
  cam->set("AcquisitionFrameRate", config_.acquisition_frame_rate);
  cam->set("ExposureAuto", config_.exposure_auto);
  cam->set("ExposureTime", config_.exposure_time);
  cam->set("AutoExposureExposureTimeUpperLimit", config_.auto_exposure_exposure_time_upper_limit);
  cam->set("Gain", config_.gain);
  cam->set("GainAuto", config_.gain_auto);
  cam->set("AutoGainLowerLimit", config_.auto_gain_lower_limit);
  cam->set("AutoGainUpperLimit", config_.auto_gain_upper_limit);
  cam->set("BlackLevel", config_.black_level);
  cam->set("BlackLevelAuto", config_.black_level_auto);
  cam->set("Gamma", config_.gamma);
  cam->set("GammaEnabled", config_.gamma_enabled);
  cam->set("Sharpness", config_.sharpness);
  cam->set("SharpnessEnabled", config_.sharpness_enabled);
  cam->set("SharpnessAuto", config_.sharpness_auto);
  cam->set("Hue", config_.hue);
  cam->set("HueEnabled", config_.hue_enabled);
  cam->set("HueAuto", config_.hue_auto);
  cam->set("Saturation", config_.saturation);
  cam->set("SaturationEnabled", config_.saturation_enabled);
  cam->set("SaturationAuto", config_.saturation_auto);
  cam->set("BalanceWhiteAuto", config_.balance_white_auto);
  cam->set("BalanceRatioSelector", std::string("Red"));
  cam->set("BalanceRatio", config_.balance_ratio_red);
  cam->set("BalanceRatioSelector", std::string("Blue"));
  cam->set("BalanceRatio", config_.balance_ratio_blue);

  // Select the Exposure End event
  cam->SetExposureEndEvent();

  // Start camera acquisition
  cam->Start();
}


};
