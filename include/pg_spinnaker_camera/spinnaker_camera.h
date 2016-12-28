#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <exception>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <sys/stat.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

enum Lines {LINE0, LINE1, LINE2, LINE3};
enum PixelFormat {MONO8, MONO12_PACKED, MONO16, RGB8, BAYER_RG8, BAYER_RG16};

class CameraManager {
 public:
  SystemPtr system;
  CameraList camList;
  CameraManager() {
    system = System::GetInstance();
    camList = system->GetCameras();
  }
  ~CameraManager() {
    camList.Clear();
    system->ReleaseInstance();
  }
  CameraPtr GetCamera(const std::string& serial_number) const {
    return camList.GetBySerial(serial_number);
  }
};

class SpinnakerCamera {
public:
  SpinnakerCamera(std::string serialNumber) {
  	CameraManager manager;
    cam_ptr_ = manager.GetCamera(serialNumber);
    cam_ptr_->Init();
    node_map_ = &cam_ptr_->GetNodeMap();
  }
  ~SpinnakerCamera() {
    cam_ptr_->DeInit();
  }

  // Print Device Info
  void PrintDeviceInfo() {
    INodeMap &device_node_map = cam_ptr_->GetTLDeviceNodeMap();
    FeatureList_t features;
    CCategoryPtr category = device_node_map.GetNode("DeviceInformation");
    if (IsAvailable(category) && IsReadable(category)) {
      category->GetFeatures(features);
      for (auto it = features.begin(); it != features.end(); ++it) {
        CNodePtr feature_node = *it;
        std::cout << feature_node->GetName() << " : ";
        CValuePtr value_ptr = (CValuePtr)feature_node;
        std::cout << (IsReadable(value_ptr) ? value_ptr->ToString() : "Node not readable");
        std::cout << std::endl;
      }
    } else {
      std::cout << "Device control information not available." << std::endl;
    }
  }

  // Start/End camera
  void Start() {
    cam_ptr_->AcquisitionMode.SetValue(AcquisitionModeEnums::AcquisitionMode_Continuous);
    cam_ptr_->BeginAcquisition();
  }
  void End() {
    cam_ptr_->EndAcquisition();
  }

  void SetPixelFormat(int format = BAYER_RG8) {
  	switch (format) {
  		case MONO8:
  			cam_ptr_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono8);
  			break;
  		case MONO12_PACKED:
  			cam_ptr_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono12Packed);
  			break;
  		case MONO16:
  			cam_ptr_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono16);
  			break;
  		case RGB8:
  			cam_ptr_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_RGB8);
  			break;
  		case BAYER_RG8:
  			cam_ptr_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_BayerRG8);
  			break;
  		case BAYER_RG16:
  			cam_ptr_->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_BayerRG16);
  			break;
  	}
  }

  void SetImageMode(int format, int width = 0, int height = 0, int binning = 1, int decimation = 1, int offset_x = 0, int offset_y = 0) {
  	this->SetPixelFormat(format);
  	if (width == 0) {
  		width = cam_ptr_->WidthMax.GetValue();
  	}
  	if (height == 0) {
  		height = cam_ptr_->HeightMax.GetValue();
  	}
  	cam_ptr_->Width.SetValue(width);
  	cam_ptr_->Height.SetValue(height);
  	cam_ptr_->OffsetX.SetValue(offset_x);
  	cam_ptr_->OffsetY.SetValue(offset_y);
  	cam_ptr_->BinningVertical.SetValue(binning);
  	cam_ptr_->DecimationVertical.SetValue(decimation);
  }

  void SetReverseX() {
  	cam_ptr_->ReverseX.SetValue(true);
  }

  void SetReverseY() {
  	cam_ptr_->ReverseY.SetValue(true);
  }

  // Setting Hardware/Software Trigger
  void EnableHardwareTrigger(int line_number) {
    this->DisableTrigger();
    switch(line_number) {
    	case LINE0:
    		cam_ptr_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line0);
			break;
    	case LINE1:
    		cam_ptr_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line1);
			break;
    	case LINE2:
    		cam_ptr_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line2);
    		break;
//    	case LINE3:
//    		cam_ptr_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line3);
//			break;
		default:
			std::cout << "Unable to set line " << line_number << "!" << std::endl;
			break;
    }
    cam_ptr_->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_On);
    cam_ptr_->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_FrameStart);
    cam_ptr_->TriggerActivation.SetValue(TriggerActivationEnums::TriggerActivation_RisingEdge);
  }
  void EnableSoftwareTrigger() {
    this->DisableTrigger();
    cam_ptr_->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
    cam_ptr_->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_On);
  }
  void DisableTrigger() {
    if (cam_ptr_->TriggerMode == NULL || cam_ptr_->TriggerMode.GetAccessMode() != RW) {
      std::cout << "Unable to disable trigger mode. Aborting..." << std::endl;
      exit(-1);
    }
    cam_ptr_->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_Off);
  }

  void EnableStrobe(int line_number) {
    switch(line_number) {
    	case LINE0:
    		cam_ptr_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line0);
			break;
    	case LINE1:
    		cam_ptr_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line1);
			break;
    	case LINE2:
    		cam_ptr_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line2);
    		break;
//    	case LINE3:
//    		cam_ptr_->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line3);
//			break;
		default:
			std::cout << "Unable to set line " << line_number << "!" << std::endl;
			break;
    }
    cam_ptr_->LineMode.SetValue(LineModeEnums::LineMode_Output);
    cam_ptr_->LineSource.SetValue(LineSourceEnums::LineSource_ExposureActive);
  }

  // Setting Black Level
  void EnableBlackLevelAuto() {
    cam_ptr_->BlackLevelAuto.SetValue(BlackLevelAutoEnums::BlackLevelAuto_Continuous);
  }
  void DisableBlackLevelAuto() {
    cam_ptr_->BlackLevelAuto.SetValue(BlackLevelAutoEnums::BlackLevelAuto_Off);
  }
  void SetBlackLevel(double black_level) {
    cam_ptr_->BlackLevel.SetValue(black_level);
  }
  double GetBlackLevel() const {
    return cam_ptr_->BlackLevel.GetValue();
  }

  // Setting Frame Rate
  void EnableFrameRateAuto() {
    CBooleanPtr AcquisitionFrameRateEnable = node_map_->GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(AcquisitionFrameRateEnable) || !IsReadable(AcquisitionFrameRateEnable)) {
      std::cout << "Unable to enable frame rate." << std::endl;
      return;
    }
    AcquisitionFrameRateEnable->SetValue(0);
  }
  void DisableFrameRateAuto() {
    CBooleanPtr AcquisitionFrameRateEnable = node_map_->GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(AcquisitionFrameRateEnable) || !IsReadable(AcquisitionFrameRateEnable)) {
      std::cout << "Unable to enable frame rate." << std::endl;
      return;
    }
    AcquisitionFrameRateEnable->SetValue(1);
  }
  void SetFrameRate(double frame_rate) {
    cam_ptr_->AcquisitionFrameRate.SetValue(frame_rate);
  }
  double GetFrameRate() const {
    return cam_ptr_->AcquisitionFrameRate.GetValue();
  }

  // Setting Exposure Time, us
  void EnableExposureAuto() {
    cam_ptr_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Continuous);
  }
  void DisableExposureAuto() {
    cam_ptr_->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
    cam_ptr_->ExposureMode.SetValue(ExposureModeEnums::ExposureMode_Timed);
  }
  void SetExposureTime(double exposure_time) {
    cam_ptr_->ExposureTime.SetValue(exposure_time);
  }
  double GetExposureTime() const {
    return cam_ptr_->ExposureTime.GetValue();
  }
  void SetExposureUpperbound(double value) {
    CFloatPtr AutoExposureExposureTimeUpperLimit = node_map_->GetNode("AutoExposureExposureTimeUpperLimit");
    AutoExposureExposureTimeUpperLimit->SetValue(value);
  }

  // Setting Gain
  void EnableGainAuto() {
    cam_ptr_->GainAuto.SetValue(GainAutoEnums::GainAuto_Continuous);
  }
  void DisableGainAuto() {
    cam_ptr_->GainAuto.SetValue(GainAutoEnums::GainAuto_Off);
  }
  void SetGain(double gain) {
    cam_ptr_->Gain.SetValue(gain);
  }
  double GetGain() const {
    return cam_ptr_->Gain.GetValue();
  }

  // Setting Gamma
  void SetGamma(double gamma) {
    cam_ptr_->Gamma.SetValue(gamma);
  }
  double GetGamma() const {
    return cam_ptr_->Gamma.GetValue();
  }

  // Setting White Balance
  void EnableWhiteBalanceAuto() {
    cam_ptr_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Continuous);
  }
  void DisableWhiteBalanceAuto() {
    cam_ptr_->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
  }
  void SetWhiteBalanceBlue(double value) {
    cam_ptr_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Blue);
    CFloatPtr BalanceRatio = node_map_->GetNode("BalanceRatio");
    BalanceRatio->SetValue(value);
  }
  void SetWhiteBalanceRed(double value) {
    cam_ptr_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Red);
    CFloatPtr BalanceRatio = node_map_->GetNode("BalanceRatio");
    BalanceRatio->SetValue(value);
  }
  double GetWhiteBalanceBlue() const {
    cam_ptr_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Blue);
    CFloatPtr BalanceRatio = node_map_->GetNode("BalanceRatio");
    return BalanceRatio->GetValue();
  }
  double GetWhiteBalanceRed() const {
    cam_ptr_->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Red);
    CFloatPtr BalanceRatio = node_map_->GetNode("BalanceRatio");
    return BalanceRatio->GetValue();
  }

  // Get Image Timestamp
  uint64_t GetSystemTimestamp() const {
    return system_timestamp_;
  }
  uint64_t GetImageTimestamp() const {
    return image_timestamp_;
  }

  // Grab Next Image
  cv::Mat GrabNextImage(const std::string& format = "bayer") {
    ImagePtr image_ptr= cam_ptr_->GetNextImage();
    system_timestamp_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    image_timestamp_ = image_ptr->GetTimeStamp();
    int width = image_ptr->GetWidth();
    int height = image_ptr->GetHeight();
    cv::Mat result;
    if (format == "bgr") {
      ImagePtr converted_image_ptr = image_ptr->Convert(PixelFormat_BGR8);
      cv::Mat temp_img(height, width, CV_8UC3, converted_image_ptr->GetData());
      result = temp_img.clone();
    } else if (format == "bayer") {
      ImagePtr converted_image_ptr = image_ptr->Convert(PixelFormat_BayerRG8);
      return cv::Mat(height, width, CV_8UC1, converted_image_ptr->GetData());
    } else if (format == "mono") {
      ImagePtr converted_image_ptr = image_ptr->Convert(PixelFormat_Mono8);
      cv::Mat temp_img(height, width, CV_8UC1, converted_image_ptr->GetData());
      result = temp_img.clone();
    } else {
      throw std::invalid_argument("Invalid argument: format = " + format + ". Expected bgr, rgr, or gray.");
    }
    image_ptr->Release();
    return result;
  }
  void TriggerSoftwareExecute() {
    cam_ptr_->TriggerSoftware.Execute();
  }

private:
  CameraPtr cam_ptr_;
  INodeMap* node_map_;
  uint64_t system_timestamp_;
  uint64_t image_timestamp_;
};