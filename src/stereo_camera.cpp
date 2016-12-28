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

namespace pg_spinnaker_camera {

StereoCamera::StereoCamera(ros::NodeHandle nh, ros::NodeHandle nhp)
: l_cam_(NULL), r_cam_(NULL), nh_(nh), nhp_(nhp), it_(nh) {

  // Get the parameters
  nhp_.param("left_serial_number", left_serial_number_, std::string("16401228"));
  nhp_.param("right_serial_number", right_serial_number_, std::string("16401229"));
  nhp_.param("left_camera_info_url", left_camera_info_url_, std::string(""));
  nhp_.param("right_camera_info_url", right_camera_info_url_, std::string(""));
}

StereoCamera::~StereoCamera() {
  if (l_cam_) {
    l_cam_->DeInit();
  }
  if (r_cam_) {
    r_cam_->DeInit();
  }
}

void StereoCamera::run() {

  // Set the image publishers before the streaming
  left_pub_  = it_.advertiseCamera("/stereo_forward/left/image_raw",  1);
  right_pub_ = it_.advertiseCamera("/stereo_forward/right/image_raw", 1);

  // Set camera info managers
  left_info_man_  = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp_, "left"),"left_optical", left_camera_info_url_));
  right_info_man_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(ros::NodeHandle(nhp_, "right"),"right_optical", right_camera_info_url_));

  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Retrieve list of cameras from the system
  CameraList cam_list = system->GetCameras();
  unsigned int num_cameras = cam_list.GetSize();

  // Check
  if (num_cameras != 2) {
    ROS_ERROR_STREAM("Incorrect number of cameras detected. Expected 2, detected " << num_cameras);
    cam_list.Clear();
    system->ReleaseInstance();
    return;
  }
  ROS_INFO("Two cameras found.");

  // Set camera configs
  for (int i = 0; i < cam_list.GetSize(); i++)
  {
    // Select camera
    CameraPtr cam = cam_list.GetByIndex(i);
    cam->Init();

    CStringPtr ptrStringSerial = cam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
    gcstring serial = ptrStringSerial->GetValue();

    if (left_serial_number_.compare(serial) == 0) {
      ROS_INFO("Configuring left camera");
      l_cam_ = cam;

      ROS_INFO("DBG 1");

      // Framerate
      // CFloatPtr acquisitionFrameRate = l_cam_->GetNodeMap().GetNode("AcquisitionFrameRate");
      // acquisitionFrameRate->SetValue(1);
      ROS_INFO("DBG 2");

      // Configure left trigger
      // CEnumerationPtr triggerMode = l_cam_->GetNodeMap().GetNode("TriggerMode");
      // triggerMode->SetIntValue(triggerMode->GetEntryByName("On")->GetValue());
      // ROS_INFO("DBG 3");
      // CEnumerationPtr triggerSource = l_cam_->GetNodeMap().GetNode("TriggerSource");
      // triggerSource->SetIntValue(triggerSource->GetEntryByName("Software")->GetValue());
      // ROS_INFO("DBG 4");
      // CEnumerationPtr triggerSelector = l_cam_->GetNodeMap().GetNode("TriggerSelector");
      // triggerSelector->SetIntValue(triggerSelector->GetEntryByName("FrameStart")->GetValue());
      // ROS_INFO("DBG 5");
      // CEnumerationPtr triggerActivation = l_cam_->GetNodeMap().GetNode("TriggerActivation");
      // triggerActivation->SetIntValue(triggerActivation->GetEntryByName("RisingEdge")->GetValue());
      // ROS_INFO("DBG 6");
      // CEnumerationPtr lineMode = l_cam_->GetNodeMap().GetNode("LineMode");
      // lineMode->SetIntValue(lineMode->GetEntryByName("On")->GetValue());
      ROS_INFO("DBG 7");
      // CEnumerationPtr lineSource = l_cam_->GetNodeMap().GetNode("LineSource");
      // lineSource->SetIntValue(lineSource->GetEntryByName("AcquisitionActive")->GetValue());
      ROS_INFO("DBG 8");
      // CEnumerationPtr lineSelector = l_cam_->GetNodeMap().GetNode("LineSelector");
      // lineSelector->SetIntValue(lineSelector->GetEntryByName("Line2")->GetValue());
      ROS_INFO("DBG 9");
      // Set acquisition mode to continuous
      CEnumerationPtr ptrAcquisitionMode = l_cam_->GetNodeMap().GetNode("AcquisitionMode");
      if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
      {
        std::cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << i << "). Aborting..." << std::endl << std::endl;
        cam_list.Clear();
        system->ReleaseInstance();
        return;
      }
      CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
      if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
      {
        std::cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << i << "). Aborting..." << std::endl << std::endl;
        cam_list.Clear();
        system->ReleaseInstance();
        return;
      }
      int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
      ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
      std::cout << "Camera " << i << " acquisition mode set to continuous..." << std::endl;

      // Start adquisition
      l_cam_->BeginAcquisition();
      ROS_INFO("DBG 10");
    }
    else if (right_serial_number_.compare(serial) == 0) {
      /*
      ROS_INFO("Configuring right camera");
      r_cam_ = cam;

      // Framerate
      // CFloatPtr acquisitionFrameRate = r_cam_->GetNodeMap().GetNode("AcquisitionFrameRate");
      // acquisitionFrameRate->SetValue(1);

      // Configure right trigger
      CEnumerationPtr triggerMode = r_cam_->GetNodeMap().GetNode("TriggerMode");
      triggerMode->SetIntValue(triggerMode->GetEntryByName("On")->GetValue());
      CEnumerationPtr triggerSource = r_cam_->GetNodeMap().GetNode("TriggerSource");
      triggerSource->SetIntValue(triggerSource->GetEntryByName("Line3")->GetValue());
      CEnumerationPtr triggerSelector = r_cam_->GetNodeMap().GetNode("TriggerSelector");
      triggerSelector->SetIntValue(triggerSelector->GetEntryByName("FrameStart")->GetValue());
      CEnumerationPtr triggerActivation = r_cam_->GetNodeMap().GetNode("TriggerActivation");
      triggerActivation->SetIntValue(triggerActivation->GetEntryByName("RisingEdge")->GetValue());

      // Set acquisition mode to continuous
      CEnumerationPtr ptrAcquisitionMode = r_cam_->GetNodeMap().GetNode("AcquisitionMode");
      if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
      {
        std::cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << i << "). Aborting..." << std::endl << std::endl;
        cam_list.Clear();
        system->ReleaseInstance();
        return;
      }
      CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
      if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
      {
        std::cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << i << "). Aborting..." << std::endl << std::endl;
        cam_list.Clear();
        system->ReleaseInstance();
        return;
      }
      int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
      ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
      std::cout << "Camera " << i << " acquisition mode set to continuous..." << std::endl;

      // Start adquisition
      r_cam_->BeginAcquisition();
      */
    }
    else {
      ROS_ERROR_STREAM("Serial number not found: " << serial << ". Expected left: " << left_serial_number_ << ". Expected right: " << right_serial_number_ << ".");
      cam_list.Clear();
      system->ReleaseInstance();
      return;
    }
  }

  while (ros::ok()) {
    ROS_INFO("DBG 11");
    ImagePtr p_result_img = l_cam_->GetNextImage();
    ROS_INFO("DBG 12");
    if (p_result_img->IsIncomplete())
    {
      ROS_WARN_STREAM("Left thread received an incomplete image (" << p_result_img->GetImageStatus() << ")");
    }
    else
    {
      // Convert image to mono 8
      ImagePtr converted_image = p_result_img->Convert(PixelFormat_Mono8, HQ_LINEAR);

      ROS_INFO("Left image received and valid");
    }
  }
  // // Launch thread
  // thread thread_left(&StereoCamera::leftFrameThread, this);
  // thread thread_right(&StereoCamera::rightFrameThread, this);
}

void StereoCamera::leftFrameThread() {
  ImagePtr p_result_img = l_cam_->GetNextImage();

  if (p_result_img->IsIncomplete())
  {
    ROS_WARN_STREAM("Left thread received an incomplete image (" << p_result_img->GetImageStatus() << ")");
  }
  else
  {
    // Convert image to mono 8
    ImagePtr converted_image = p_result_img->Convert(PixelFormat_Mono8, HQ_LINEAR);

    ROS_INFO("Left image received and valid");
  }
}

void StereoCamera::rightFrameThread() {
  ImagePtr p_result_img = r_cam_->GetNextImage();

  if (p_result_img->IsIncomplete())
  {
    ROS_WARN_STREAM("Right thread received an incomplete image (" << p_result_img->GetImageStatus() << ")");
  }
  else
  {
    // Convert image to mono 8
    ImagePtr converted_image = p_result_img->Convert(PixelFormat_Mono8, HQ_LINEAR);

    ROS_INFO("Right image received and valid");
  }
}


};
