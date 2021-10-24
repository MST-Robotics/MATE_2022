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
static const char* configFile = "/boot/frc.json";

// Create namespace variables, stucts, and objects.
unsigned int team;
bool server = false;

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

vector<CameraConfig> cameraConfigs;
vector<SwitchedCameraConfig> switchedCameraConfigs;
vector<UsbCamera> cameras;
vector<CvSink> cameraSinks;
vector<CvSource> cameraSources;

raw_ostream& ParseError() 
{
	return errs() << "config error in '" << configFile << "': ";
}

/****************************************************************************
		Description:	Read camera config file from the web dashboard.

		Arguments: 		CONST JSON&

		Returns: 		BOOL
****************************************************************************/
bool ReadCameraConfig(const json& config) 
{
	// Create instance variables.
	CameraConfig camConfig;

	// Get camera name.
	try 
	{
		camConfig.name = config.at("name").get<string>();
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Could not read camera name: " << e.what() << "\n";
		return false;
	}

	// Get camera path.
	try 
	{
		camConfig.path = config.at("path").get<string>();
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Camera '" << camConfig.name << "': could not read path: " << e.what() << "\n";
		return false;
	}

	// Get stream properties.
	if (config.count("stream") != 0)
	{
		camConfig.streamConfig = config.at("stream");
	}

	camConfig.config = config;

	cameraConfigs.emplace_back(move(camConfig));
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
	error_code errorCode;
	raw_fd_istream is(configFile, errorCode);
	if (errorCode) 
	{
		errs() << "Could not open '" << configFile << "': " << errorCode.message() << "\n";
		return false;
	}

	// Parse file.
	json parseFile;
	try 
	{
		parseFile = json::parse(is);
	} 
	catch (const json::parse_error& e) 
	{
		ParseError() << "Byte " << e.byte << ": " << e.what() << "\n";
		return false;
	}

	// Check if the top level is an object.
	if (!parseFile.is_object()) 
	{
		ParseError() << "Must be JSON object!" << "\n";
		return false;
	}

	// Get team number.
	try 
	{
		team = parseFile.at("team").get<unsigned int>();
	} 
	catch (const json::exception& e) 
	{
		ParseError() << "Could not read team number: " << e.what() << "\n";
		return false;
	}

	// Get NetworkTable mode.
	if (parseFile.count("ntmode") != 0) 
	{
		try 
		{
			auto str = parseFile.at("ntmode").get<string>();
			StringRef s(str);
			if (s.equals_lower("client")) 
			{
				server = false;
			} 
			else 
			{
				if (s.equals_lower("server")) 
				{
					server = true;
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
		for (auto&& camera : parseFile.at("cameras")) 
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
void StartCamera(const CameraConfig& config) 
{
	// Print debug
	cout << "Starting camera '" << config.name << "' on " << config.path << "\n";

	// Create new CameraServer instance and start camera.
	CameraServer* instance = CameraServer::GetInstance();
	UsbCamera camera{config.name, config.path};
	MjpegServer server = instance->StartAutomaticCapture(camera);

	// Set camera parameters.
	camera.SetConfigJson(config.config);
	camera.SetConnectionStrategy(VideoSource::kConnectionKeepOpen);

	// Check for unexpected parameters.
	if (config.streamConfig.is_object())
	{
		server.SetConfigJson(config.streamConfig); 
	}

	// Store the camera video in a vector. (so we can access it later)
	CvSink cvSink = instance->GetVideo(config.name);
	CvSource cvSource = instance->PutVideo(config.name + "Processed", 640, 480);
	cameras.emplace_back(camera);
	cameraSinks.emplace_back(cvSink);
	cameraSources.emplace_back(cvSource);
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
		configFile = argv[1];
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
	if (server) 
	{
		cout << "Setting up NetworkTables server" << "\n";
		NetworkTablesInstance.StartServer();
	} 
	else 
	{
		cout << "Setting up NetworkTables client for team " << team << "\n";
		NetworkTablesInstance.StartClientTeam(team);
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
	for (const auto& config : cameraConfigs)
	{
		StartCamera(config);
	}

	/**************************************************************************
	 			Start Image Processing on Camera 0
	 * ************************************************************************/
	if (cameraSinks.size() >= 1) 
	{
		// Create object pointers for threads.
		VideoGet VideoGetter;
		VideoProcess VideoProcessor;
		VideoShow VideoShower;

		// Preallocate image objects.
		Mat	frame(480, 640, CV_8U, 1);
		Mat finalImg(480, 640, CV_8U, 1);

		// Create a global instance of mutex to protect it.
		shared_timed_mutex MutexGet;
		shared_timed_mutex MutexShow;

		// Vision options and values.
		int targetCenterX = 0;
		int targetCenterY = 0;
		double contrastValue = 0;
		double targetAngle = 0;
		bool cameraSourceIndex = false;
		bool tuningMode = false;
		bool drivingMode = false;
		bool trackingMode = true;
		bool enableSolvePNP = false;
		vector<int> trackbarValues {1, 255, 1, 255, 1, 255};
		vector<double> solvePNPValues {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Start multi-threading.
		thread VideoGetThread(&VideoGet::StartCapture, &VideoGetter, ref(frame), ref(cameraSourceIndex), ref(drivingMode), ref(MutexGet));
		thread VideoProcessThread(&VideoProcess::Process, &VideoProcessor, ref(frame), ref(finalImg), ref(targetCenterX), ref(targetCenterY), ref(contrastValue), ref(targetAngle), ref(tuningMode), ref(drivingMode), ref(trackingMode), ref(enableSolvePNP), ref(trackbarValues), ref(solvePNPValues), ref(VideoGetter), ref(MutexGet), ref(MutexShow));
		thread VideoShowerThread(&VideoShow::ShowFrame, &VideoShower, ref(finalImg), ref(cameraSources), ref(MutexShow));

		while (1)
		{
			try
			{
				// Check if any of the threads have stopped.
				if (!VideoGetter.GetIsStopped() && !VideoProcessor.GetIsStopped() && !VideoShower.GetIsStopped())
				{
					// Get NetworkTables data.
					cameraSourceIndex = NetworkTable->GetBoolean("Camera Source", false);
					tuningMode = NetworkTable->GetBoolean("Tuning Mode", false);
					drivingMode = NetworkTable->GetBoolean("Driving Mode", false);
					trackingMode = NetworkTable->GetBoolean("Pipe Tracking Mode", true);
					enableSolvePNP = NetworkTable->GetBoolean("Enable SolvePNP", false);
					contrastValue = NetworkTable->GetNumber("Contrast Value", 1211.0);
					trackbarValues[0] = int(NetworkTable->GetNumber("HMN", 1));
					trackbarValues[1] = int(NetworkTable->GetNumber("HMX", 255));
					trackbarValues[2] = int(NetworkTable->GetNumber("SMN", 1));
					trackbarValues[3] = int(NetworkTable->GetNumber("SMX", 255));
					trackbarValues[4] = int(NetworkTable->GetNumber("VMN", 1));
					trackbarValues[5] = int(NetworkTable->GetNumber("VMX", 255));

					// Put NetworkTables data.
					NetworkTable->PutNumber("Target Center X", targetCenterX);
					NetworkTable->PutNumber("Target Center Y", targetCenterY);
					NetworkTable->PutNumber("Target Angle", (targetAngle + int(NetworkTable->GetNumber("X Setpoint Offset", 0))));
					NetworkTable->PutNumber("SPNP X Dist", solvePNPValues[0]);
					NetworkTable->PutNumber("SPNP Y Dist", solvePNPValues[1]);
					NetworkTable->PutNumber("SPNP Z Dist", solvePNPValues[2]);
					NetworkTable->PutNumber("SPNP Roll", solvePNPValues[3]);
					NetworkTable->PutNumber("SPNP Pitch", solvePNPValues[4]);
					NetworkTable->PutNumber("SPNP Yaw", solvePNPValues[5]);

					// Put different trackbar values on smartdashboard depending on camera source.
					static bool setValuesToggle = false;
					if (!cameraSourceIndex && setValuesToggle == false)		// Turret Camera.
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
						setValuesToggle = true;
					}
					else
					{
						if (cameraSourceIndex && setValuesToggle == true)	// Bottom Camera.
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
							setValuesToggle = false;
						}
					}
					

					// Sleep.
					this_thread::sleep_for(std::chrono::milliseconds(20));

					// Print debug info.
					//cout << "Getter FPS: " << VideoGetter.GetFPS() << "\n";
					//cout << "Processor FPS: " << VideoProcessor.GetFPS() << "\n";
					//cout << "Shower FPS: " << VideoShower.GetFPS() << "\n";
				}
				else
				{
					// Notify other threads the program is stopping.
					VideoGetter.SetIsStopping(true);
					VideoProcessor.SetIsStopping(true);
					VideoShower.SetIsStopping(true);
					break;
				}
			}
			catch (const exception& e)
			{
				cout << "CRITICAL: A main thread error has occured!" << "\n";
			}
		}

		// Stop all threads.
		VideoGetThread.join();
		VideoProcessThread.join();
		VideoShowerThread.join();

		// Print that program has safely and successfully shutdown.
		cout << "All threads have been released! Program will now stop..." << "\n";
		
		// Kill program.
		return EXIT_SUCCESS;
	}
}