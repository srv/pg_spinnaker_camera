#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <ros/ros.h>
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

	SpinnakerCamera(std::string serial_number) : serial_(serial_number), acquiring_(false){
		system_ = Spinnaker::System::GetInstance();
		cam_list_ = system_->GetCameras();
		unsigned int num_cameras = cam_list_.GetSize();
		ROS_INFO_STREAM("[pg_spinnaker_camera]: Number of cameras detected: " << num_cameras);
		cam_ptr_ = cam_list_.GetBySerial(serial_number);
		if (cam_ptr_) {
			cam_ptr_->Init();
			node_map_ = &cam_ptr_->GetNodeMap();
		} else {
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: FAILED to open camera " << serial_number << ". Is it connected?");
		}
	}

	~SpinnakerCamera() {
		ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Destructor called.");
		if (cam_ptr_) end();

		// Clear camera list before releasing system
		cam_list_.Clear();
    // Release system
		system_->ReleaseInstance();
	}

	bool isConnected() {
		if (cam_ptr_) return true;
		else return false;
	}

	// Print Device Info
	void printDeviceInfo() {
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
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ")  Device control information not available.");
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
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Enumeration name " << property_name << " not implemented.");
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(enumerationPtr)) {
			if (Spinnaker::GenApi::IsWritable(enumerationPtr)) {
				Spinnaker::GenApi::CEnumEntryPtr enumEmtryPtr = enumerationPtr->GetEntryByName(entry_name.c_str());
				if (Spinnaker::GenApi::IsAvailable(enumEmtryPtr)) {
					if (Spinnaker::GenApi::IsReadable(enumEmtryPtr)) {
						enumerationPtr->SetIntValue(enumEmtryPtr->GetValue());
						ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") " << property_name << " set to " << enumerationPtr->GetCurrentEntry()->GetSymbolic() << ".");
						return true;
					} else {
						ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Entry name " << entry_name << " not writable.");
					}
				} else {
					ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Entry name " << entry_name << " not available.");
				}
			} else {
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Enumeration " << property_name << " not writable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Enumeration " << property_name << " not available.");
		}
		return false;
	}

	bool set(const std::string &property_name, const float &value) {
		Spinnaker::GenApi::CFloatPtr floatPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(floatPtr)) {
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(floatPtr)) {
			if (Spinnaker::GenApi::IsWritable(floatPtr)) {
				floatPtr->SetValue(value);
				ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") " << property_name << " set to " << floatPtr->GetValue() << ".");
				return true;
			} else {
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not writable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not available.");
		}
		return false;
	}

	bool set(const std::string &property_name, const bool &value) {
		Spinnaker::GenApi::CBooleanPtr boolPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(boolPtr)) {
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(boolPtr)) {
			if (Spinnaker::GenApi::IsWritable(boolPtr)) {
				boolPtr->SetValue(value);
				ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") " << property_name << " set to " << boolPtr->GetValue() << ".");
				return true;
			} else {
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not writable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not available.");
		}
		return false;
	}

	bool set(const std::string &property_name, const int &value) {
		Spinnaker::GenApi::CIntegerPtr intPtr = node_map_->GetNode(property_name.c_str());
		if (!Spinnaker::GenApi::IsImplemented(intPtr)) {
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
			return false;
		}
		if (Spinnaker::GenApi::IsAvailable(intPtr)) {
			if (Spinnaker::GenApi::IsWritable(intPtr)) {
				intPtr->SetValue(value);
				ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") " << property_name << " set to " << intPtr->GetValue() << ".");
				return true;
			} else {
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not writable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not available.");
		}
		return false;
	}

	bool setMaxInt(const std::string &property_name) {
		Spinnaker::GenApi::CIntegerPtr intPtr = node_map_->GetNode(property_name.c_str());
		if (Spinnaker::GenApi::IsAvailable(intPtr)) {
			if (Spinnaker::GenApi::IsWritable(intPtr)) {
				intPtr->SetValue(intPtr->GetMax());
				ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") " << property_name << " set to " << intPtr->GetValue() << ".");
				return true;
			} else {
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not writable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not available.");
		}
		return false;
	}

	void execute(const std::string& command) {
		Spinnaker::GenApi::CCommandPtr commandPtr = node_map_->GetNode(command.c_str());
		if (!Spinnaker::GenApi::IsAvailable(commandPtr) || !Spinnaker::GenApi::IsWritable(commandPtr))
		{
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") unable to command " << command << ". Aborting...");
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
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not readable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not available.");
		}
		return -1;
	}

	int getInteger(const std::string& property_name) {
		Spinnaker::GenApi::CIntegerPtr integerPtr = node_map_->GetNode("property_name");
		if (Spinnaker::GenApi::IsAvailable(integerPtr)) {
			if (Spinnaker::GenApi::IsReadable(integerPtr)) {
				return integerPtr->GetValue();
			} else {
				ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not readable.");
			}
		} else {
			ROS_WARN_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Feature " << property_name << " not available.");
		}
		return -1;
	}

	// Start/stop acquisition camera
	void startAcquisition() {
		std::lock_guard<std::mutex> lock(mutex_);
		acquiring_ = true;
		cam_ptr_->BeginAcquisition();
		ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Acquisition started.");
	}

	void stopAcquisition(bool enable_mutex=true) {
		acquiring_ = false;
		if (enable_mutex) {
			std::lock_guard<std::mutex> lock(mutex_);
			cam_ptr_->EndAcquisition();
		} else {
			cam_ptr_->EndAcquisition();
		}
		ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Acquisition stop.");
	}

	bool isAcquiring() {
		return acquiring_;
	}

	void end() {
		std::lock_guard<std::mutex> lock(mutex_);

		ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Camera end called.");
		try {
			stopAcquisition(false);
      cam_ptr_->DeInit();
      ROS_INFO_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") DeInit.");
		} catch (Spinnaker::Exception &e) {
      std::cerr << "[ERROR]: " << e.what() << std::endl;
      ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ")" << e.what());
    }
    delete cam_ptr_;
	}

	// Get Image Timestamp
	uint64_t getSystemTimestamp() const {
		return system_timestamp_;
	}

	uint64_t getImageTimestamp() const {
		return image_timestamp_;
	}

	// Grab Next Image
	cv::Mat grabNextImage() {
		std::lock_guard<std::mutex> lock(mutex_);
		if (!acquiring_) return cv::Mat();

		Spinnaker::ImagePtr image_ptr = cam_ptr_->GetNextImage();
		std::string format(image_ptr->GetPixelFormatName());

		system_timestamp_ = std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		image_timestamp_ = image_ptr->GetTimeStamp();

		cv::Mat result;
		if (image_ptr->IsIncomplete()) {
			ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Received and incomplete image.");
		} else {
			int width = image_ptr->GetWidth();
			int height = image_ptr->GetHeight();

			if (format == "BayerRG8") {
				cv::Mat temp_img(height, width, CV_8UC1, image_ptr->GetData());
				result = temp_img;
			} else {
				ROS_ERROR_STREAM("[pg_spinnaker_camera]: (" << serial_ << ") Invalid image format.");
				//throw std::invalid_argument("Invalid image format = " + format + ". BayerRG8.");
			}
		}
		image_ptr->Release();
		return result;
	}

private:
	Spinnaker::CameraPtr cam_ptr_;
	Spinnaker::GenApi::INodeMap *node_map_;
	Spinnaker::SystemPtr system_;
	Spinnaker::CameraList cam_list_;
	uint64_t system_timestamp_;
	uint64_t image_timestamp_;
	std::string serial_;
	std::mutex mutex_;
	bool acquiring_;
};
