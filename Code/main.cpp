#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>

#include "Headers/VideoGet.h"
#include "Headers/VideoProcess.h"
#include "Headers/VideoShow.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>
#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <cameraserver/CameraServer.h>

using namespace cv;
using namespace cs;
using namespace nt;
using namespace frc;
using namespace wpi;
using namespace std;

/*
	JSON format:
	{
		"team": <team number>,
		"ntmode": <"client" or "server", "client" if unspecified>
		"cameras": [
			{
				"name": <camera name>
				"path": <path, e.g. "/dev/video0">
				"pixel format": <"MJPEG", "YUYV", etc>	// optional
				"width": <video mode width>			  // optional
				"height": <video mode height>			// optional
				"fps": <video mode fps>				  // optional
				"brightness": <percentage brightness>	// optional
				"white balance": <"auto", "hold", value> // optional
				"exposure": <"auto", "hold", value>	  // optional
				"properties": [						  // optional
					{
						"name": <property name>
						"value": <property value>
					}
				],
				"stream": {							  // optional
					"properties": [
						{
							"name": <stream property name>
							"value": <stream property value>
						}
					]
				}
			}
		]
		"switched cameras": [
			{
				"name": <virtual camera name>
				"key": <network table key used for selection>
				// if NT value is a string, it's treated as a name
				// if NT value is a double, it's treated as an integer index
			}
		]
	}
 */

// Store config file.
static const char* m_cConfigFile = "/boot/frc.json";

// Create namespace variables, stucts, and objects.
unsigned int m_nTeam;
bool m_bServer = false;

struct CameraConfig 
{
	string name;
	string path;
	json config;
	json streamConfig;
};

struct SwitchedCameraConfig 
{
	string name;
	string key;
};

vector<CameraConfig> m_vCameraConfigs;
vector<SwitchedCameraConfig> m_vSwitchedCameraConfigs;
vector<UsbCamera> m_vCameras;
vector<CvSink> m_vCameraSinks;
vector<CvSource> m_vCameraSources;

raw_ostream& ParseError() 
{
	return errs() << "config error in '" << m_cConfigFile << "': ";
}

/****************************************************************************
		Description:	Read camera config file from the web dashboard.

		Arguments: 		CONST JSON&

		Returns: 		BOOL
****************************************************************************/
bool ReadCameraConfig(const json& fConfig) 
{
	// Create instance variables.
	CameraConfig m_pCamConfig;

	// Get camera name.
	try 
	{
		m_pCamConfig.name = fConfig.at("name").get<string>();
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Could not read camera name: " << e.what() << "\n";
		return false;
	}

	// Get camera path.
	try 
	{
		m_pCamConfig.path = fConfig.at("path").get<string>();
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Camera '" << m_pCamConfig.name << "': could not read path: " << e.what() << "\n";
		return false;
	}

	// Get stream properties.
	if (fConfig.count("stream") != 0)
	{
		m_pCamConfig.streamConfig = fConfig.at("stream");
	}

	m_pCamConfig.config = fConfig;

	m_vCameraConfigs.emplace_back(move(m_pCamConfig));
	return true;
}

/****************************************************************************
		Description:	Read config file from the web dashboard.

		Arguments: 		None

		Returns: 		BOOL
****************************************************************************/
bool ReadConfig() 
{
	// Open config file.
	error_code m_pErrorCode;
	raw_fd_istream is(m_cConfigFile, m_pErrorCode);
	if (m_pErrorCode) 
	{
		errs() << "Could not open '" << m_cConfigFile << "': " << m_pErrorCode.message() << "\n";
		return false;
	}

	// Parse file.
	json m_fParseFile;
	try 
	{
		m_fParseFile = json::parse(is);
	} 
	catch (const json::parse_error& e) 
	{
		ParseError() << "Byte " << e.byte << ": " << e.what() << "\n";
		return false;
	}

	// Check if the top level is an object.
	if (!m_fParseFile.is_object()) 
	{
		ParseError() << "Must be JSON object!" << "\n";
		return false;
	}

	// Get team number.
	try 
	{
		m_nTeam = m_fParseFile.at("team").get<unsigned int>();
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Could not read team number: " << e.what() << "\n";
		return false;
	}

	// Get NetworkTable mode.
	if (m_fParseFile.count("ntmode") != 0) 
	{
		try 
		{
			auto str = m_fParseFile.at("ntmode").get<string>();
			StringRef s(str);
			if (s.equals_lower("client")) 
			{
				m_bServer = false;
			} 
			else 
			{
				if (s.equals_lower("server")) 
				{
					m_bServer = true;
				}
				else 
				{
					ParseError() << "Could not understand ntmode value '" << str << "'" << "\n";
				}
			} 
		} 
		catch (const json::exception& e) 
		{
			ParseError() << "Could not read ntmode: " << e.what() << "\n";
		}
	}

	// Read camera configs and get cameras.
	try 
	{
		for (auto&& camera : m_fParseFile.at("cameras")) 
		{
			if (!ReadCameraConfig(camera))
			{
				return false;
			}
		}
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Could not read cameras: " << e.what() << "\n";
		return false;
	}

	return true;
}

/****************************************************************************
		Description:	Starts cameras and camera streams.

		Arguments: 		CONST CAMERACONFIG&

		Returns: 		Nothing
****************************************************************************/
void StartCamera(const CameraConfig& fConfig) 
{
	// Print debug
	cout << "Starting camera '" << fConfig.name << "' on " << fConfig.path << "\n";

	// Create new CameraServer instance and start camera.
	CameraServer* m_Inst = CameraServer::GetInstance();
	UsbCamera m_Camera{fConfig.name, fConfig.path};
	MjpegServer m_pServer = m_Inst->StartAutomaticCapture(m_Camera);

	// Set camera parameters.
	m_Camera.SetConfigJson(fConfig.config);
	m_Camera.SetConnectionStrategy(VideoSource::kConnectionKeepOpen);

	// Check for unexpected parameters.
	if (fConfig.streamConfig.is_object())
	{
		m_pServer.SetConfigJson(fConfig.streamConfig); 
	}

	// Store the camera video in a vector. (so we can access it later)
	CvSink m_cvSink = m_Inst->GetVideo(fConfig.name);
	CvSource m_cvSource = m_Inst->PutVideo(fConfig.name + "Processed", 640, 480);
	m_vCameras.emplace_back(m_Camera);
	m_vCameraSinks.emplace_back(m_cvSink);
	m_vCameraSources.emplace_back(m_cvSource);
}

/****************************************************************************
    Description:	Main method

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
int main(int argc, char* argv[]) 
{
	/************************************************************************** 
	  			Read Configurations
	 * ************************************************************************/
	if (argc >= 2) 
	{
		m_cConfigFile = argv[1];
	}

	if (!ReadConfig())
	{
		return EXIT_FAILURE;
	}

	/**************************************************************************
	  			Start NetworkTables
	 * ************************************************************************/
	// Create instance.
	auto NetworkTablesInstance = NetworkTableInstance::GetDefault();
	auto NetworkTable = NetworkTablesInstance.GetTable("SmartDashboard");

	// Start Networktables as a client or server.
	if (m_bServer) 
	{
		cout << "Setting up NetworkTables server" << "\n";
		NetworkTablesInstance.StartServer();
	} 
	else 
	{
		cout << "Setting up NetworkTables client for team " << m_nTeam << "\n";
		NetworkTablesInstance.StartClientTeam(m_nTeam);
	}

	// Populate NetworkTables.
	NetworkTable->PutBoolean("Camera Source", false);
	NetworkTable->PutBoolean("Tuning Mode", false);
	NetworkTable->PutBoolean("Driving Mode", false);
	NetworkTable->PutBoolean("Pipe Tracking Mode", true);
	NetworkTable->PutBoolean("Enable SolvePNP", false);
	NetworkTable->PutNumber("X Setpoint Offset", 0);
	NetworkTable->PutNumber("Contrast Value", 1211);
	NetworkTable->PutNumber("HMN", 48);
	NetworkTable->PutNumber("HMX", 104);
	NetworkTable->PutNumber("SMN", 0);
	NetworkTable->PutNumber("SMX", 128);
	NetworkTable->PutNumber("VMN", 0);
	NetworkTable->PutNumber("VMX", 0);

	/**************************************************************************
	 			Start Cameras
	 * ************************************************************************/
	for (const auto& config : m_vCameraConfigs)
	{
		StartCamera(config);
	}

	/**************************************************************************
	 			Start Image Processing on Camera 0
	 * ************************************************************************/
	if (m_vCameraSinks.size() >= 1) 
	{
		// Create object pointers for threads.
		VideoGet m_pVideoGetter;
		VideoProcess m_pVideoProcessor;
		VideoShow m_pVideoShower;

		// Preallocate image objects.
		Mat	m_pFrame(480, 640, CV_8U, 1);
		Mat m_pFinalImg(480, 640, CV_8U, 1);

		// Create a global instance of mutex to protect it.
		shared_timed_mutex m_pMutexGet;
		shared_timed_mutex m_pMutexShow;

		// Vision options and values.
		int m_nTargetCenterX = 0;
		int m_nTargetCenterY = 0;
		double m_dContrastValue = 0;
		double m_dTargetAngle = 0;
		bool m_bCameraSourceIndex = false;
		bool m_bTuningMode = false;
		bool m_bDrivingMode = false;
		bool m_bTrackingMode = true;
		bool m_bEnableSolvePNP = false;
		vector<int> m_vTrackbarValues {1, 255, 1, 255, 1, 255};
		vector<double> m_vSolvePNPValues {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Start multi-threading.
		thread m_pVideoGetThread(&VideoGet::StartCapture, &m_pVideoGetter, ref(m_pFrame), ref(m_bCameraSourceIndex), ref(m_bDrivingMode), ref(m_pMutexGet));
		thread m_pVideoProcessThread(&VideoProcess::Process, &m_pVideoProcessor, ref(m_pFrame), ref(m_pFinalImg), ref(m_nTargetCenterX), ref(m_nTargetCenterY), ref(m_dContrastValue), ref(m_dTargetAngle), ref(m_bTuningMode), ref(m_bDrivingMode), ref(m_bTrackingMode), ref(m_bEnableSolvePNP), ref(m_vTrackbarValues), ref(m_vSolvePNPValues), ref(m_pVideoGetter), ref(m_pMutexGet), ref(m_pMutexShow));
		thread m_pVideoShowerThread(&VideoShow::ShowFrame, &m_pVideoShower, ref(m_pFinalImg), ref(m_vCameraSources), ref(m_pMutexShow));

		while (1)
		{
			try
			{
				// Check if any of the threads have stopped.
				if (!m_pVideoGetter.GetIsStopped() && !m_pVideoProcessor.GetIsStopped() && !m_pVideoShower.GetIsStopped())
				{
					// Get NetworkTables data.
					m_bCameraSourceIndex = NetworkTable->GetBoolean("Camera Source", false);
					m_bTuningMode = NetworkTable->GetBoolean("Tuning Mode", false);
					m_bDrivingMode = NetworkTable->GetBoolean("Driving Mode", false);
					m_bTrackingMode = NetworkTable->GetBoolean("Pipe Tracking Mode", true);
					m_bEnableSolvePNP = NetworkTable->GetBoolean("Enable SolvePNP", false);
					m_dContrastValue = NetworkTable->GetNumber("Contrast Value", 1211.0);
					m_vTrackbarValues[0] = int(NetworkTable->GetNumber("HMN", 1));
					m_vTrackbarValues[1] = int(NetworkTable->GetNumber("HMX", 255));
					m_vTrackbarValues[2] = int(NetworkTable->GetNumber("SMN", 1));
					m_vTrackbarValues[3] = int(NetworkTable->GetNumber("SMX", 255));
					m_vTrackbarValues[4] = int(NetworkTable->GetNumber("VMN", 1));
					m_vTrackbarValues[5] = int(NetworkTable->GetNumber("VMX", 255));

					// Put NetworkTables data.
					NetworkTable->PutNumber("Target Center X", m_nTargetCenterX);
					NetworkTable->PutNumber("Target Center Y", m_nTargetCenterY);
					NetworkTable->PutNumber("Target Angle", (m_dTargetAngle + int(NetworkTable->GetNumber("X Setpoint Offset", 0))));
					NetworkTable->PutNumber("SPNP X Dist", m_vSolvePNPValues[0]);
					NetworkTable->PutNumber("SPNP Y Dist", m_vSolvePNPValues[1]);
					NetworkTable->PutNumber("SPNP Z Dist", m_vSolvePNPValues[2]);
					NetworkTable->PutNumber("SPNP Roll", m_vSolvePNPValues[3]);
					NetworkTable->PutNumber("SPNP Pitch", m_vSolvePNPValues[4]);
					NetworkTable->PutNumber("SPNP Yaw", m_vSolvePNPValues[5]);

					// Put different trackbar values on smartdashboard depending on camera source.
					static bool bSetValuesToggle = false;
					if (!m_bCameraSourceIndex && bSetValuesToggle == false)		// Turret Camera.
					{
						// Put trackbar values for tape tracking.
						NetworkTable->PutNumber("HMN", 48);
						NetworkTable->PutNumber("HMX", 104);
						NetworkTable->PutNumber("SMN", 0);
						NetworkTable->PutNumber("SMX", 128);
						NetworkTable->PutNumber("VMN", 0);
						NetworkTable->PutNumber("VMX", 0);

						// Set tracking mode.
						NetworkTable->PutBoolean("Pipe Tracking Mode", true);

						// Set toggle.
						bSetValuesToggle = true;
					}
					else
					{
						if (m_bCameraSourceIndex && bSetValuesToggle == true)	// Bottom Camera.
						{
							// Put trackbar values for tape tracking.
							NetworkTable->PutNumber("HMN", 0);
							NetworkTable->PutNumber("HMX", 0);
							NetworkTable->PutNumber("SMN", 0);
							NetworkTable->PutNumber("SMX", 0);
							NetworkTable->PutNumber("VMN", 0);
							NetworkTable->PutNumber("VMX", 0);

							// Set tracking mode.
							NetworkTable->PutBoolean("Pipe Tracking Mode", false);

							// Set toggle.
							bSetValuesToggle = false;
						}
					}
					

					// Sleep.
					this_thread::sleep_for(std::chrono::milliseconds(20));

					// Print debug info.
					//cout << "Getter FPS: " << m_pVideoGetter.GetFPS() << "\n";
					//cout << "Processor FPS: " << m_pVideoProcessor.GetFPS() << "\n";
					//cout << "Shower FPS: " << m_pVideoShower.GetFPS() << "\n";
				}
				else
				{
					// Notify other threads the program is stopping.
					m_pVideoGetter.SetIsStopping(true);
					m_pVideoProcessor.SetIsStopping(true);
					m_pVideoShower.SetIsStopping(true);
					break;
				}
			}
			catch (const exception& e)
			{
				cout << "CRITICAL: A main thread error has occured!" << "\n";
			}
		}

		// Stop all threads.
		m_pVideoGetThread.join();
		m_pVideoProcessThread.join();
		m_pVideoShowerThread.join();

		// Print that program has safely and successfully shutdown.
		cout << "All threads have been released! Program will now stop..." << "\n";
		
		// Kill program.
		return EXIT_SUCCESS;
	}
}