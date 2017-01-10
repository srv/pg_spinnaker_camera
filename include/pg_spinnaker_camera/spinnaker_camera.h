#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <exception>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <sys/stat.h>

enum Lines {
	LINE0, LINE1, LINE2, LINE3
};
enum PixelFormat {
	MONO8, MONO12_PACKED, MONO16, RGB8, BAYER_RG8, BAYER_RG16
};

class SpinnakerCamera {
public:

	// Nested class to hide from implementation
	class DeviceEventHandler : public Spinnaker::DeviceEvent {
	 public:
		DeviceEventHandler(SpinnakerCamera* cam) : cam_(cam) {};
		~DeviceEventHandler(){};
		// Once Exposure End Event is detected, the OnDeviceEvent function will be called
		void OnDeviceEvent(Spinnaker::GenICam::gcstring eventName) {
			std::thread t(cam_->callback_);
			t.detach();
		}
	 private:
	 	SpinnakerCamera* cam_;
	};

	SpinnakerCamera(std::string serial_number) : event_handler_(new DeviceEventHandler(this)), serial_(serial_number), is_end_(false){
		system_ = Spinnaker::System::GetInstance();
		cam_list_ = system_->GetCameras();
		unsigned int num_cameras = cam_list_.GetSize();
		std::cout << "Number of cameras detected: " << num_cameras << std::endl << std::endl;
		cam_ptr_ = cam_list_.GetBySerial(serial_number);
		if (cam_ptr_) {
			cam_ptr_->Init();
			node_map_ = &cam_ptr_->GetNodeMap();
			setCallback(std::bind(&SpinnakerCamera::callback, this));
		} else {
			std::cout << "FAILED to open camera " << serial_number << ". Is it connected?" << std::endl;
		}
	}

	~SpinnakerCamera() {
		std::cout << "[INFO]:  (" << serial_ << ") Destructor called" << std::endl;
		if (cam_ptr_) End();

		// Clear camera list before releasing system
		cam_list_.Clear();
    // Release system
		system_->ReleaseInstance();
	}

	bool IsConnected() {
		if (cam_ptr_) return true;
		else return false;
	}

	// Print Device Info
	void PrintDeviceInfo() {
		if (!cam_ptr_) return;
		Spinnaker::GenApi::INodeMap &device_node_map = cam_ptr_->GetTLDeviceNodeMap();
		Spinnaker::GenApi::FeatureList_t features;
		Spinnaker::GenApi::CCategoryPtr category = device_node_map.GetNode("DeviceInformation");
		if (Spinnaker::GenApi::IsAvailable(category) && Spinnaker::GenApi::IsReadable(category)) {
			category->GetFeatures(features);
			for (auto it = features.begin(); it != features.end(); ++it) {
				Spinnaker::GenApi::CNodePtr feature_node = *it;
				std::cout << feature_node->GetName() << " : ";
				Spinnaker::GenApi::CValuePtr value_ptr = (Spinnaker::GenApi::CValuePtr) feature_node;
				std::cout << (Spinnaker::GenApi::IsReadable(value_ptr) ? value_ptr->ToString() : "Node not readable");
				std::cout << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Device control information not available." << std::endl;
		}
	}

	bool set(const std::string &property_name, const std::string &entry_name) {
		// *** NOTES ***
		// Enumeration nodes are slightly more complicated to set than other
		// nodes. This is because setting an enumeration node requires working
		// with two nodes instead of the usual one.
		//
		// As such, there are a number of steps to setting an enumeration node:
		// retrieve the enumeration node from the nodemap, retrieve the desired
		// entry node from the enumeration node, retrieve the integer value from
		// the entry node, and set the new value of the enumeration node with
		// the integer value from the entry node.
		// std::cout << "Setting " << property_name << " to " << entry_name << " (enum)" << std::endl;
		Spinnaker::GenApi::CEnumerationPtr enumerationPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(enumerationPtr)) {
			std::cout << "[ERROR]: (" << serial_ << ")  Enumeration name " << property_name << " not implemented" << std::endl;
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(enumerationPtr)) {
			if (Spinnaker::GenApi::IsWritable(enumerationPtr)) {
				Spinnaker::GenApi::CEnumEntryPtr enumEmtryPtr = enumerationPtr->GetEntryByName(entry_name.c_str());
				if (Spinnaker::GenApi::IsAvailable(enumEmtryPtr)) {
					if (Spinnaker::GenApi::IsReadable(enumEmtryPtr)) {
						enumerationPtr->SetIntValue(enumEmtryPtr->GetValue());
						std::cout << "[INFO]: " << property_name << " set to " << enumerationPtr->GetCurrentEntry()->GetSymbolic() << "..." << std::endl;
						return true;
					} else {
						std::cout << "[ERROR]: (" << serial_ << ")  Entry name " << entry_name << " not writable" << std::endl;
					}
				} else {
					std::cout << "[ERROR]: (" << serial_ << ")  Entry name " << entry_name << " not available" << std::endl;
				}
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Enumeration " << property_name << " not writable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Enumeration " << property_name << " not available" << std::endl;
		}
		return false;
	}

	bool set(const std::string &property_name, const float &value) {
		// std::cout << "Setting " << property_name << " to " << value << " (float)" << std::endl;
		Spinnaker::GenApi::CFloatPtr floatPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(floatPtr)) {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature name " << property_name << " not implemented" << std::endl;
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(floatPtr)) {
			if (Spinnaker::GenApi::IsWritable(floatPtr)) {
				floatPtr->SetValue(value);
				std::cout << "[INFO]: " << property_name << " set to " << floatPtr->GetValue() << "..." << std::endl;
				return true;
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not writable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not available" << std::endl;
		}
		return false;
	}

	bool set(const std::string &property_name, const bool &value) {
		// std::cout << "Setting " << property_name << " to " << value << " (bool)" << std::endl;
		Spinnaker::GenApi::CBooleanPtr boolPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(boolPtr)) {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature name " << property_name << " not implemented" << std::endl;
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(boolPtr)) {
			if (Spinnaker::GenApi::IsWritable(boolPtr)) {
				boolPtr->SetValue(value);
				std::cout << "[INFO]: " << property_name << " set to " << boolPtr->GetValue() << "..." << std::endl;
				return true;
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not writable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not available" << std::endl;
		}
		return false;
	}

	bool set(const std::string &property_name, const int &value) {
		// std::cout << "Setting " << property_name << " to " << value << " (int)" << std::endl;
		Spinnaker::GenApi::CIntegerPtr intPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(intPtr)) {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature name " << property_name << " not implemented" << std::endl;
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(intPtr)) {
			if (Spinnaker::GenApi::IsWritable(intPtr)) {
				intPtr->SetValue(value);
				std::cout << "[INFO]: " << property_name << " set to " << intPtr->GetValue() << "..." << std::endl;
				return true;
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not writable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not available" << std::endl;
		}
		return false;
	}

	bool setMaxInt(const std::string &property_name) {
		Spinnaker::GenApi::CIntegerPtr intPtr = node_map_->GetNode(property_name.c_str());
		if (Spinnaker::GenApi::IsAvailable(intPtr)) {
			if (Spinnaker::GenApi::IsWritable(intPtr)) {
				// std::cout << "Setting " << property_name << " to " << intPtr->GetMax() << " (int)" << std::endl;
				intPtr->SetValue(intPtr->GetMax());
				std::cout << "[INFO]: " << property_name << " set to " << intPtr->GetValue() << "..." << std::endl;
				return true;
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not writable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not available" << std::endl;
		}
		return false;
	}

	void execute(const std::string& command) {
		Spinnaker::GenApi::CCommandPtr commandPtr = node_map_->GetNode(command.c_str());
		if (!Spinnaker::GenApi::IsAvailable(commandPtr) || !Spinnaker::GenApi::IsWritable(commandPtr))
		{
			std::cout << "Unable to command " << command << ". Aborting..." << std::endl;
		} else {
			commandPtr->Execute();
		}
	}

	float getFloat(const std::string& property_name) {
		Spinnaker::GenApi::CFloatPtr floatPtr = node_map_->GetNode("property_name");
		if (Spinnaker::GenApi::IsAvailable(floatPtr)) {
			if (Spinnaker::GenApi::IsReadable(floatPtr)) {
				return floatPtr->GetValue();
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not readable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not available" << std::endl;
		}
		return -1;
	}

	int getInteger(const std::string& property_name) {
		Spinnaker::GenApi::CIntegerPtr integerPtr = node_map_->GetNode("property_name");
		if (Spinnaker::GenApi::IsAvailable(integerPtr)) {
			if (Spinnaker::GenApi::IsReadable(integerPtr)) {
				return integerPtr->GetValue();
			} else {
				std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not readable" << std::endl;
			}
		} else {
			std::cout << "[ERROR]: (" << serial_ << ")  Feature " << property_name << " not available" << std::endl;
		}
		return -1;
	}

	void SetExposureEndEvent() {
		// Select the Exposure End event
		set("EventSelector", std::string("ExposureEnd"));
		// Turn on the Event notification for Exposure End Event
		set("EventNotification", std::string("On"));

		// Register event handler
		cam_ptr_->RegisterEvent(*event_handler_, "EventExposureEnd");
	}

	// Start/End camera
	void Start() {
		is_end_ = false;
		set("AcquisitionMode", std::string("Continuous"));
		cam_ptr_->BeginAcquisition();
	}

	void End() {
		std::lock_guard<std::mutex> lock(mutex_);
		is_end_ = true;

		std::cout << "[INFO]:  (" << serial_ << ") Camera end called" << std::endl;
		// Select the Exposure End event
		//set("EventSelector", std::string("ExposureEnd"));
		// Turn on the Event notification for Exposure End Event
		//set("EventNotification", std::string("Off"));
		try {
			//cam_ptr_->UnregisterEvent(*event_handler_);
			//delete event_handler_;
      std::cout << "[INFO]:  (" << serial_ << ") Device event handler unregistered" << std::endl;
			cam_ptr_->EndAcquisition();
      std::cout << "[INFO]:  (" << serial_ << ") Acquisition stopped" << std::endl;
      cam_ptr_->DeInit();
      std::cout << "[INFO]:  (" << serial_ << ") DeInit" << std::endl;
		} catch (Spinnaker::Exception &e) {
      std::cout << "Error: " << e.what() << std::endl;
    }
    delete cam_ptr_;
	}

	void SetPixelFormat(int format = BAYER_RG8) {
		switch (format) {
			case MONO8:
				set("PixelFormat", std::string("Mono8"));
				break;
			case MONO12_PACKED:
				set("PixelFormat", std::string("Mono12Packed"));
				break;
			case MONO16:
				set("PixelFormat", std::string("Mono16"));
				break;
			case RGB8:
				set("PixelFormat", std::string("RGB8"));
				break;
			case BAYER_RG8:
				set("PixelFormat", std::string("BayerRG8"));
				break;
			case BAYER_RG16:
				set("PixelFormat", std::string("BayerRG16"));
				break;
		}
	}

	void SetImageMode(int format = BAYER_RG8,
										int width = 0,
										int height = 0,
										int binning = 1,
										int decimation = 1,
										int offset_x = 0,
										int offset_y = 0) {
		this->SetPixelFormat(format);
		if (width == 0) {
			setMaxInt("Width");
		} else {
			set("Width", width);
		}
		if (height == 0) {
			setMaxInt("Height");
		} else {
			set("Height", height);
		}
		set("OffsetX", offset_x);
		set("OffsetY", offset_y);
		set("BinningVertical", binning);
		set("DecimationVertical", decimation);
	}

	void SetReverseX() {
		set("ReverseX", true);
	}

	void SetReverseY() {
		set("ReverseY", true);
	}

	// Setting Hardware/Software Trigger
	void EnableHardwareTrigger(int line_number) {
		set("TriggerMode", std::string("Off"));
		switch (line_number) {
			case LINE0:
				set("TriggerSource", std::string("Line0"));
				break;
			case LINE1:
				set("TriggerSource", std::string("Line1"));
				break;
			case LINE2:
				set("TriggerSource", std::string("Line2"));
				break;
	   	case LINE3:
	   		set("TriggerSource", std::string("Line3"));
				break;
			default:
				std::cout << "[ERROR]: (" << serial_ << ") Unable to set line " << line_number << "!" << std::endl;
				break;
		}
		set("TriggerMode", std::string("On"));
		set("TriggerSelector", std::string("FrameStart"));
		set("TriggerActivation", std::string("RisingEdge"));
	}

	void EnableSoftwareTrigger() {
		set("TriggerMode", std::string("Off"));
		set("TriggerSource", std::string("Software"));
		set("TriggerMode", std::string("On"));
	}

	void DisableTrigger() {
		set("TriggerMode", std::string("Off"));
	}

	void EnableStrobe(int line_number) {
		switch (line_number) {
			case LINE0:
				set("LineSelector", std::string("Line0"));
				break;
			case LINE1:
				set("LineSelector", std::string("Line1"));
				break;
			case LINE2:
				set("LineSelector", std::string("Line2"));
				break;
   		case LINE3:
				set("LineSelector", std::string("Line3"));
				break;
			default:
				std::cout << "[ERROR]: (" << serial_ << ") Unable to set line " << line_number << "!" << std::endl;
				break;
		}
		set("LineMode", std::string("Output"));
		set("LineSource", std::string("ExposureActive"));
	}

	// Setting Black Level
	void EnableBlackLevelAuto() {
		set("BlackLevelAuto", std::string("Continuous"));
	}

	void DisableBlackLevelAuto() {
		set("BlackLevelAuto", std::string("Off"));
	}

	void SetBlackLevel(float black_level) {
		set("BlackLevel", black_level);
	}

	double GetBlackLevel() {
		return getFloat("BlackLevel");
	}

	// Setting Frame Rate
	void EnableFrameRateAuto() {
		set("AcquisitionFrameRateAuto", std::string("Continuous"));
	}

	void DisableFrameRateAuto() {
		set("AcquisitionFrameRateAuto", std::string("Off"));
	}

	void SetFrameRate(float frame_rate) {
		set("AcquisitionFrameRate", frame_rate);
	}

	double GetFrameRate() {
		return getFloat("AcquisitionFrameRate");
	}

	// Setting Exposure Time, us
	void EnableExposureAuto() {
		set("ExposureAuto", std::string("Continuous"));
	}

	void DisableExposureAuto() {
		set("ExposureAuto", std::string("Off"));
		set("ExposureMode", std::string("Timed"));
	}

	void SetExposureTime(float exposure_time) {
		set("ExposureTime", exposure_time);
	}

	double GetExposureTime() {
		return getFloat("ExposureTime");
	}

	void SetExposureUpperbound(float value) {
		set("AutoExposureExposureTimeUpperLimit", value);
	}

	// Setting Gain
	void EnableGainAuto() {
		set("GainAuto", std::string("Continuous"));
	}

	void DisableGainAuto() {
		set("GainAuto", std::string("Off"));
	}

	void SetGain(float gain) {
		set("Gain", gain);
	}

	float GetGain() {
		return getFloat("Gain");
	}

	// Setting Gamma
	void SetGamma(float gamma) {
		set("Gamma", gamma);
	}

	float GetGamma() {
		return getFloat("Gamma");
	}

	// Setting White Balance
	void EnableWhiteBalanceAuto() {
		set("BalanceWhiteAuto", std::string("Continuous"));
	}

	void DisableWhiteBalanceAuto() {
		set("BalanceWhiteAuto", std::string("Off"));
	}

	void SetWhiteBalanceBlue(float value) {
		set("BalanceRatioSelector", std::string("Blue"));
		set("BalanceRatio", value);
	}

	void SetWhiteBalanceRed(float value) {
		set("BalanceRatioSelector", std::string("Red"));
		set("BalanceRatio", value);
	}

	float GetWhiteBalanceBlue() {
		set("BalanceRatioSelector", std::string("Blue"));
		return getFloat("BalanceRatio");
	}

	float GetWhiteBalanceRed() {
		set("BalanceRatioSelector", std::string("Red"));
		return getFloat("BalanceRatio");
	}

	// Get Image Timestamp
	uint64_t GetSystemTimestamp() const {
		return system_timestamp_;
	}

	uint64_t GetImageTimestamp() const {
		return image_timestamp_;
	}

	// Grab Next Image
	cv::Mat GrabNextImage() {
		std::lock_guard<std::mutex> lock(mutex_);
		if (is_end_) return cv::Mat();

		Spinnaker::ImagePtr image_ptr = cam_ptr_->GetNextImage();
		std::string format(image_ptr->GetPixelFormatName());

		system_timestamp_ = std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		image_timestamp_ = image_ptr->GetTimeStamp();

		cv::Mat result;
		if (image_ptr->IsIncomplete()) {
			std::cout << "[ERROR]: (" << serial_ << ")  Received and incomplete image " << std::endl;
		} else {
			int width = image_ptr->GetWidth();
			int height = image_ptr->GetHeight();

			if (format == "BayerRG8") {
				cv::Mat temp_img(height, width, CV_8UC1, image_ptr->GetData());
				result = temp_img;
			} else {
				std::cout << "Invalid image format." << std::endl;
				//throw std::invalid_argument("Invalid image format = " + format + ". BayerRG8.");
			}
		}
		image_ptr->Release();
		return result;
	}

	void TriggerSoftwareExecute() {
		execute("TriggerSoftware");
	}

	void callback() {
		std::cout << "(" << serial_ << ") This is an empty callback..." << std::endl;
	}

	void setCallback(std::function<void (void)> f) {
    callback_ = f;
  }

private:
	Spinnaker::CameraPtr cam_ptr_;
	Spinnaker::GenApi::INodeMap *node_map_;
	Spinnaker::SystemPtr system_;
	Spinnaker::CameraList cam_list_;
	DeviceEventHandler* event_handler_;
	std::function<void (void)> callback_;
	uint64_t system_timestamp_;
	uint64_t image_timestamp_;
	std::string serial_;
	std::mutex mutex_;
	bool is_end_;
};
