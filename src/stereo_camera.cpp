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
: l_cam_(NULL), r_cam_(NULL), left_counter_(0), right_counter_(0),
  nh_(nh), nhp_(nhp), it_(nh), exec_stop_(false) {
  // Camera info
  nhp_.param("left_serial_number", left_serial_number_,
    std::string("16401228"));
  nhp_.param("right_serial_number", right_serial_number_,
    std::string("16401229"));
  nhp_.param("left_camera_info_url", left_camera_info_url_, std::string(""));
  nhp_.param("right_camera_info_url", right_camera_info_url_, std::string(""));

  // Sync
  nhp_.param("imgs_buffer_size", imgs_buffer_size_, 2);
  nhp_.param("max_sec_diff", max_sec_diff_, 0.05);

  // Camera config
  nhp_.getParam("binning_vertical", config_.binning_vertical);
  nhp_.getParam("decimation_vertical", config_.decimation_vertical);
  nhp_.getParam("pixel_format", config_.pixel_format);
  nhp_.getParam("video_mode", config_.video_mode);
  nhp_.getParam("line_selector", config_.line_selector);
  nhp_.getParam("line_mode", config_.line_mode);
  nhp_.getParam("line_source", config_.line_source);
  nhp_.getParam("trigger_source", config_.trigger_source);
  nhp_.getParam("trigger_selector", config_.trigger_selector);
  nhp_.getParam("trigger_activation", config_.trigger_activation);
  nhp_.getParam("acquisition_frame_rate_auto",
    config_.acquisition_frame_rate_auto);
  nhp_.getParam("acquisition_frame_rate_enabled",
    config_.acquisition_frame_rate_enabled);
  nhp_.getParam("acquisition_frame_rate", config_.acquisition_frame_rate);
  nhp_.getParam("exposure_auto", config_.exposure_auto);
  nhp_.getParam("exposure_time", config_.exposure_time);
  nhp_.getParam("auto_exposure_exposure_time_upper_limit",
    config_.auto_exposure_exposure_time_upper_limit);
  nhp_.getParam("gain_auto", config_.gain_auto);
  nhp_.getParam("gain", config_.gain);
  nhp_.getParam("auto_gain_lower_limit", config_.auto_gain_lower_limit);
  nhp_.getParam("auto_gain_upper_limit", config_.auto_gain_upper_limit);
  nhp_.getParam("balance_white_auto", config_.balance_white_auto);
  nhp_.getParam("balance_ratio_red", config_.balance_ratio_red);
  nhp_.getParam("balance_ratio_blue", config_.balance_ratio_blue);
  nhp_.getParam("black_level", config_.black_level);
  nhp_.getParam("gamma_enabled", config_.gamma_enabled);
  nhp_.getParam("hue_enabled", config_.hue_enabled);
  nhp_.getParam("gamma", config_.gamma);
  nhp_.getParam("hue", config_.hue);
  nhp_.getParam("saturation", config_.saturation);
  nhp_.getParam("flip_left", config_.flip_left);
  nhp_.getParam("flip_right", config_.flip_right);

  l_cam_ = std::shared_ptr<SpinnakerCamera>(
    new SpinnakerCamera(left_serial_number_));
  r_cam_ = std::shared_ptr<SpinnakerCamera>(
    new SpinnakerCamera(right_serial_number_));

  if (l_cam_->isConnected()) {
    ROS_INFO_STREAM("[pg_spinnaker_camera]: Left camera " <<
      left_serial_number_ << " connected!");
    if (r_cam_->isConnected()) {
      ROS_INFO_STREAM("[pg_spinnaker_camera]: Right camera " <<
        right_serial_number_ << " connected!");
      ROS_INFO_STREAM("[pg_spinnaker_camera]: ------- Configuring " <<
        " LEFT camera -------");
      configureCamera(l_cam_, true);
      ROS_INFO("\n");
      ROS_INFO_STREAM("[pg_spinnaker_camera]: ------- Configuring " <<
        " RIGHT camera -------");
      configureCamera(r_cam_, false);
    } else {
      ROS_ERROR_STREAM("[pg_spinnaker_camera]: Right camera " <<
        right_serial_number_ << " not connected!");
    }
  } else {
    ROS_ERROR_STREAM("[pg_spinnaker_camera]: Left camera " <<
      left_serial_number_ << " not connected!");
  }
}

void StereoCamera::stop() {
  exec_stop_ = true;
  if (l_cam_) {
    l_cam_->end();
  }
  if (r_cam_) {
    r_cam_->end();
  }
}

void StereoCamera::run() {
  if (!l_cam_->isConnected() && !r_cam_->isConnected()) return;

  // Set the image publishers before the streaming
  left_pub_  = it_.advertiseCamera("/stereo_forward/left/image_raw",  1);
  right_pub_ = it_.advertiseCamera("/stereo_forward/right/image_raw", 1);

  // Set camera info managers
  left_info_man_  = std::shared_ptr<camera_info_manager::CameraInfoManager>(
    new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp_, "left"),
      "left_optical", left_camera_info_url_));
  right_info_man_ = std::shared_ptr<camera_info_manager::CameraInfoManager>(
    new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp_, "right"),
      "right_optical", right_camera_info_url_));

  // Start acquisition
  if (!l_cam_->isAcquiring())
    l_cam_->startAcquisition();
  if (!r_cam_->isAcquiring())
    r_cam_->startAcquisition();

  // Start threads
  std::thread thread_left(&StereoCamera::leftFrameThread, this);
  std::thread thread_right(&StereoCamera::rightFrameThread, this);
  thread_left.join();
  thread_right.join();
}

void StereoCamera::leftFrameThread() {
  while (!exec_stop_) {
    // Process only when subscriptors
    if (left_pub_.getNumSubscribers() == 0) {
      continue;
    }

    ros::Time ros_time = ros::Time::now();
    cv::Mat left_img = l_cam_->grabNextImage();

    if (left_img.rows > 0) {
      if (left_pub_.getNumSubscribers() > 0) {
        // Setup header
        std_msgs::Header header;
        header.seq = left_counter_;
        header.stamp = ros_time;

        // Flip?
        std::string enc = sensor_msgs::image_encodings::BAYER_RGGB8;
        if (config_.flip_left) {
          cv::flip(left_img, left_img, -1);
          enc = sensor_msgs::image_encodings::BAYER_BGGR8;
        }

        // Setup image
        sensor_msgs::Image img;
        cv_bridge::CvImage img_bridge =
          cv_bridge::CvImage(header, enc, left_img);
        img_bridge.toImageMsg(img);

        // Camera info
        sensor_msgs::CameraInfo lci = left_info_man_->getCameraInfo();
        lci.header.stamp = ros_time;

        // Adjust full-res ROI to binning ROI
        int bin_dec = std::max(config_.binning_vertical, config_.decimation_vertical);

        // Set the operational parameters in CameraInfo (binning, ROI)
        lci.binning_x = bin_dec;
        lci.binning_y = bin_dec;
        // ROI in CameraInfo is in unbinned coordinates, need to scale up
        lci.roi.x_offset = 0;
        lci.roi.y_offset = 0;
        lci.roi.height = 1536; // FIXME: read from parameter file
        lci.roi.width = 2048; // FIXME: read from parameter file
        lci.roi.do_rectify = false;

        // Set frame id
        img.header.frame_id = lci.header.frame_id;

        // Publish
        if (right_pub_.getNumSubscribers() == 0) {
          // No right subscribers
          left_pub_.publish(img, lci);
        } else {
          // Right sync
          std::lock_guard<std::mutex> lock(r_sync_mutex_);

          // Search a time coincidence with right
          int idx_r = -1;
          for (uint i=0; i < r_imgs_buffer_.size(); i++) {
            double r_stamp = r_imgs_buffer_[i].header.stamp.toSec();
            if (fabs(r_stamp - ros_time.toSec()) < max_sec_diff_) {
              idx_r = static_cast<int>(i);
              break;
            }
          }
          if (idx_r >= 0) {
            // Get the corresponding right image
            sensor_msgs::Image r_img = r_imgs_buffer_[idx_r];

            // Publish left/right
            sensor_msgs::CameraInfo rci = right_info_man_->getCameraInfo();
            r_img.header.stamp = ros_time;
            lci.header.stamp = ros_time;
            rci.header.stamp = ros_time;
            left_pub_.publish(img, lci);
            right_pub_.publish(r_img, rci);

            // Delete this right image from buffer
            r_imgs_buffer_.erase(r_imgs_buffer_.begin(),
              r_imgs_buffer_.begin() + idx_r + 1);
          } else {
            // Add the left image to the buffer
            std::lock_guard<std::mutex> lock(l_sync_mutex_);
            if (l_imgs_buffer_.size() >= imgs_buffer_size_) {
              l_imgs_buffer_.erase(l_imgs_buffer_.begin(),
                l_imgs_buffer_.begin() + 1);
            }
            l_imgs_buffer_.push_back(img);
          }
        }
      }
    } else {
      ROS_ERROR("[pg_spinnaker_camera]: LEFT image incomplete.");
    }
    left_counter_++;
  }
}

void StereoCamera::rightFrameThread() {
  while (!exec_stop_) {
    // Process only when subscriptors
    if (right_pub_.getNumSubscribers() == 0) {
      continue;
    }

    ros::Time ros_time = ros::Time::now();
    cv::Mat right_img = r_cam_->grabNextImage();

    if (right_img.rows > 0) {
      if (right_pub_.getNumSubscribers() > 0) {
        // Setup header
        std_msgs::Header header;
        header.seq = right_counter_;
        header.stamp = ros_time;

        // Flip?
        std::string enc = sensor_msgs::image_encodings::BAYER_RGGB8;
        if (config_.flip_right) {
          cv::flip(right_img, right_img, -1);
          enc = sensor_msgs::image_encodings::BAYER_BGGR8;
        }

        // Setup image
        sensor_msgs::Image img;
        cv_bridge::CvImage img_bridge =
          cv_bridge::CvImage(header, enc, right_img);
        img_bridge.toImageMsg(img);

        // Camera info
        sensor_msgs::CameraInfo rci = right_info_man_->getCameraInfo();
        rci.header.stamp = ros_time;

        // Adjust full-res ROI to binning ROI
        int bin_dec = std::max(config_.binning_vertical, config_.decimation_vertical);

        // Set the operational parameters in CameraInfo (binning, ROI)
        rci.binning_x = bin_dec;
        rci.binning_y = bin_dec;
        // ROI in CameraInfo is in unbinned coordinates, need to scale up
        rci.roi.x_offset = 0;
        rci.roi.y_offset = 0;
        rci.roi.height = 1536; // FIXME: read from parameter file
        rci.roi.width = 2048; // FIXME: read from parameter file
        rci.roi.do_rectify = false;

        // Set frame id
        img.header.frame_id = rci.header.frame_id;

        // Publish
        if (left_pub_.getNumSubscribers() == 0) {
          // No right subscribers
          right_pub_.publish(img, rci);
        } else {
          // Left sync
          std::lock_guard<std::mutex> lock(l_sync_mutex_);

          // Search a time coincidence with left
          int idx_l = -1;
          for (uint i=0; i < l_imgs_buffer_.size(); i++) {
            double l_stamp = l_imgs_buffer_[i].header.stamp.toSec();
            if (fabs(l_stamp - ros_time.toSec()) < max_sec_diff_) {
              idx_l = static_cast<int>(i);
              break;
            }
          }
          if (idx_l >= 0) {
            // Get the corresponding left image
            sensor_msgs::Image l_img = l_imgs_buffer_[idx_l];

            // Publish left/right
            sensor_msgs::CameraInfo lci = left_info_man_->getCameraInfo();
            l_img.header.stamp = ros_time;
            rci.header.stamp = ros_time;
            lci.header.stamp = ros_time;
            right_pub_.publish(img, rci);
            left_pub_.publish(l_img, lci);

            // Delete this left image from buffer
            l_imgs_buffer_.erase(l_imgs_buffer_.begin(),
              l_imgs_buffer_.begin() + idx_l + 1);
          } else {
            // Add the right image to the buffer
            std::lock_guard<std::mutex> lock(r_sync_mutex_);
            if (r_imgs_buffer_.size() >= imgs_buffer_size_) {
              r_imgs_buffer_.erase(r_imgs_buffer_.begin(),
                r_imgs_buffer_.begin() + 1);
            }
            r_imgs_buffer_.push_back(img);
          }
        }
      }
    } else {
      ROS_ERROR("[pg_spinnaker_camera]: RIGHT image incomplete.");
    }
    right_counter_++;
  }
}

void StereoCamera::configureCamera(const std::shared_ptr<SpinnakerCamera>& cam,
                                   bool is_left) {
  // Image format
  cam->set("BinningVertical", config_.binning_vertical);
  cam->set("DecimationVertical", config_.decimation_vertical);
  cam->set("PixelFormat", config_.pixel_format);
  cam->set("AcquisitionMode", std::string("Continuous"));
  cam->set("VideoMode", config_.video_mode);

  if (is_left) {
    // Set strobe (LEFT CAMERA ONLY)
    // and AcquisitionFrameRate (only if Trigger Mode is Off)
    cam->set("LineSelector", config_.line_selector);
    cam->set("LineMode", config_.line_mode);
    cam->set("LineSource", config_.line_source);
    cam->set("TriggerMode", std::string("Off"));
    cam->set("AcquisitionFrameRateAuto", config_.acquisition_frame_rate_auto);
    cam->set("AcquisitionFrameRateEnabled",
      config_.acquisition_frame_rate_enabled);
    cam->set("AcquisitionFrameRate", config_.acquisition_frame_rate);
  } else {
    // Trigger (Must be switched off to change source!)
    // (RIGHT CAMERA ONLY)
    cam->set("TriggerMode", std::string("Off"));
    cam->set("TriggerSource", config_.trigger_source);
    cam->set("TriggerMode", std::string("On"));
    cam->set("TriggerSelector", config_.trigger_selector);
    cam->set("TriggerActivation", config_.trigger_activation);
  }

  cam->set("ExposureMode", std::string("Timed"));
  cam->set("ExposureAuto", config_.exposure_auto);
  if (config_.exposure_auto.compare(std::string("Off")) == 0) {
    cam->set("ExposureTime", config_.exposure_time);
  }
  cam->set("AutoExposureTimeUpperLimit",
    config_.auto_exposure_exposure_time_upper_limit);

  cam->set("GainAuto", config_.gain_auto);
  if (config_.gain_auto.compare(std::string("Off")) == 0) {
    cam->set("Gain", config_.gain);
  }
  cam->set("AutoGainLowerLimit", config_.auto_gain_lower_limit);
  cam->set("AutoGainUpperLimit", config_.auto_gain_upper_limit);

  cam->set("BlackLevel", config_.black_level);

  if (config_.gamma_enabled) {
    cam->set("GammaEnabled", config_.gamma_enabled);
    cam->set("Gamma", config_.gamma);
  }
  if (config_.hue_enabled) {
    cam->set("HueEnabled", config_.hue_enabled);
    cam->set("Hue", config_.hue);
  }
  if (config_.saturation_enabled) {
    cam->set("SaturationEnabled", config_.saturation_enabled);
    cam->set("Saturation", config_.saturation);
  }
}


};
