#include <cstdio>
#include <chrono>
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
#include "Headers/rapidjson/filereadstream.h"
#include "Headers/rapidjson/filewritestream.h"
#include "Headers/rapidjson/writer.h"
#include "Headers/rapidjson/document.h"

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
using namespace rapidjson;

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
static const char* visionTuningFile = "/home/pi/UnderwaterRoboticsVision/MATE_2022/Code/trackbar_values.json";

// Create namespace variables, stucts, and objects.
unsigned int team;
bool server = false;

// Create json interactable object.
Document visionTuningJSON;

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
			wpi::StringRef s(str);
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
		Description:	Gets values from json file and updates networktables
						with the corresponding values.

		Arguments: 		AUTO &NetworkTable, STRING selectionState

		Returns: 		Nothing
****************************************************************************/
void GetJSONValues(auto &NetworkTable, string selectionState)
{
	// Convert string parameter to char array.
	char* state = &*selectionState.begin();

	// Get corresponding object from JSON file based on tracking state.
	const rapidjson::Value& object = visionTuningJSON[state];
	int contourAreaMinLimit = object["ContourAreaMinLimit"].GetInt();
	int contourAreaMaxLimit = object["ContourAreaMaxLimit"].GetInt();
	int hmn = object["HMN"].GetInt();
	int hmx = object["HMX"].GetInt();
	int smn = object["SMN"].GetInt();
	int smx = object["SMX"].GetInt();
	int vmn = object["VMN"].GetInt();
	int vmx = object["VMX"].GetInt();

	// Update network tables with the values from the JSON document object.
	NetworkTable->PutNumber("Contour Area Min Limit", contourAreaMinLimit);
	NetworkTable->PutNumber("Contour Area Max Limit", contourAreaMaxLimit);
	NetworkTable->PutNumber("HMN", hmn);
	NetworkTable->PutNumber("HMX", hmx);
	NetworkTable->PutNumber("SMN", smn);
	NetworkTable->PutNumber("SMX", smx);
	NetworkTable->PutNumber("VMN", vmn);
	NetworkTable->PutNumber("VMX", vmx);
}

/****************************************************************************
		Description:	Gets values from networktables and update the JSON file
						with the corresponding values.

		Arguments: 		AUTO &NetworkTable, STRING selectionState

		Returns: 		Nothing
****************************************************************************/
void PutJSONValues(auto &NetworkTable, string selectionState)
{
	// Convert string parameter to char array.
	char* state = &*selectionState.begin();

	// Get corresponding object from JSON file based on tracking state.
	rapidjson::Value& object = visionTuningJSON[state];
	int contourAreaMinLimit = NetworkTable->GetNumber("Contour Area Min Limit", 0);
	int contourAreaMaxLimit = NetworkTable->GetNumber("Contour Area Max Limit", 0);
	int hmn = NetworkTable->GetNumber("HMN", 0);
	int hmx = NetworkTable->GetNumber("HMX", 0);
	int smn = NetworkTable->GetNumber("SMN", 0);
	int smx = NetworkTable->GetNumber("SMX", 0);
	int vmn = NetworkTable->GetNumber("VMN", 0);
	int vmx = NetworkTable->GetNumber("VMX", 0);

	// Update network tables with the values from the JSON document object.
	object["ContourAreaMinLimit"].SetInt(contourAreaMinLimit);
	object["ContourAreaMaxLimit"].SetInt(contourAreaMaxLimit);
	object["HMN"].SetInt(hmn);
	object["HMX"].SetInt(hmx);
	object["SMN"].SetInt(smn);
	object["SMX"].SetInt(smx);
	object["VMN"].SetInt(vmn);
	object["VMX"].SetInt(vmx);
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
	// Set web dashboard config path if given as argument.
	if (argc >= 2) 
	{
		configFile = argv[1];
	}

	// Read dashboard config.
	if (!ReadConfig())
	{
		return EXIT_FAILURE;
	}

	// Open vision trackbar json for reading and writing.
	FILE* jsonFile = fopen(visionTuningFile, "r");
	// Check if file was successfully opened.
	if (jsonFile == nullptr)
	{
		return EXIT_FAILURE;
	}
	
	// Create empty data buffer.
	char readBuffer[65536];
	// Store opened file in buffer.
	FileReadStream readFileStream(jsonFile, readBuffer, sizeof(readBuffer));
	// Parse stream buffer into rapidjson document.
	visionTuningJSON.ParseStream(readFileStream);

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
	NetworkTable->PutBoolean("Write JSON", false);
	NetworkTable->PutBoolean("Camera Source", false);
	NetworkTable->PutBoolean("Tuning Mode", false);
	NetworkTable->PutBoolean("Driving Mode", false);
	NetworkTable->PutBoolean("Trench Tracking Mode", false);
	NetworkTable->PutBoolean("Line Tracking Mode", false);
	NetworkTable->PutBoolean("Fish Tracking Mode", true);
	NetworkTable->PutBoolean("Tape Tracking Mode", false);
	NetworkTable->PutBoolean("Take Shapshot", false);
	NetworkTable->PutBoolean("Enable SolvePNP", false);
	NetworkTable->PutNumber("X Setpoint Offset", 0);
	NetworkTable->PutNumber("Contour Area Min Limit", 1211);
	NetworkTable->PutNumber("Contour Area Max Limit", 2000);
	NetworkTable->PutNumber("Center Line Tolerance", 50);
	NetworkTable->PutNumber("HMN", 48);
	NetworkTable->PutNumber("HMX", 104);
	NetworkTable->PutNumber("SMN", 0);
	NetworkTable->PutNumber("SMX", 128);
	NetworkTable->PutNumber("VMN", 0);
	NetworkTable->PutNumber("VMX", 0);
	NetworkTable->PutNumberArray("Tracking Results", vector<double> {});

	/**************************************************************************
	 			Start Cameras
	**************************************************************************/
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
		int centerLineTolerance = 0;
		double contourAreaMinLimit = 0;
		double contourAreaMaxLimit = 0;
		bool writeJSON = false;
		bool cameraSourceIndex = false;
		bool tuningMode = false;
		bool drivingMode = false;
		bool takeShapshot = false;
		bool enableSolvePNP = false;
		bool valsSet = false;
		int trackingMode = VideoProcess::FISH_TRACKING;
		enum SelectionStates { TRENCH, LINE, FISH, TAPE };
		int selectionState = FISH;
		vector<int> trackbarValues {1, 255, 1, 255, 1, 255};
		vector<double> trackingResults {};
		vector<double> solvePNPValues {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Start multi-threading.
		thread VideoGetThread(&VideoGet::StartCapture, &VideoGetter, ref(frame), ref(cameraSourceIndex), ref(drivingMode), ref(MutexGet));
		thread VideoProcessThread(&VideoProcess::Process, &VideoProcessor, ref(frame), ref(finalImg), ref(targetCenterX), ref(targetCenterY), ref(centerLineTolerance), ref(contourAreaMinLimit), ref(contourAreaMaxLimit), ref(tuningMode), ref(drivingMode), ref(trackingMode), ref(takeShapshot), ref(enableSolvePNP), ref(trackbarValues), ref(trackingResults), ref(solvePNPValues), ref(VideoGetter), ref(MutexGet), ref(MutexShow));
		thread VideoShowerThread(&VideoShow::ShowFrame, &VideoShower, ref(finalImg), ref(cameraSources), ref(MutexShow));

		while (1)
		{
			try
			{
				// Check if any of the threads have stopped.
				if (!VideoGetter.GetIsStopped() && !VideoProcessor.GetIsStopped() && !VideoShower.GetIsStopped())
				{
					// Get NetworkTables data.
					writeJSON = NetworkTable->GetBoolean("Write JSON", false);
					cameraSourceIndex = NetworkTable->GetBoolean("Camera Source", false);
					tuningMode = NetworkTable->GetBoolean("Tuning Mode", false);
					drivingMode = NetworkTable->GetBoolean("Driving Mode", false);
					bool trenchMode = NetworkTable->GetBoolean("Trench Tracking Mode", false);
					bool lineMode = NetworkTable->GetBoolean("Line Tracking Mode", false);
					bool fishMode = NetworkTable->GetBoolean("Fish Tracking Mode", true);
					bool tapeMode = NetworkTable->GetBoolean("Tape Tracking Mode", false);
					// Tracking mode selection state logic.
					switch (selectionState)
					{
						case TRENCH:
							// If line mode is selected move to other state.
							if (lineMode)
							{
								// Deselect trench tracking mode.
								NetworkTable->PutBoolean("Trench Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::LINE_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "TRENCH");
								// Move to other state.
								selectionState = LINE;
							}
							// If fish mode is selected move to other state.
							else if (fishMode)
							{
								// Deselect trench tracking mode.
								NetworkTable->PutBoolean("Trench Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::FISH_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "TRENCH");
								// Move to other state.
								selectionState = FISH;
							}
							// If tape mode is selected move to other state.
							else if (tapeMode)
							{
								// Deselect trench tracking mode.
								NetworkTable->PutBoolean("Trench Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::TAPE_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "TRENCH");
								// Move to other state.
								selectionState = TAPE;
							}
							else
							{
								// Make sure trench mode is true while in this state.
								NetworkTable->PutBoolean("Trench Tracking Mode", true);
								// Only set mode specific values once.
								if (!valsSet)
								{
									// Update networktables values.
									GetJSONValues(NetworkTable, "TRENCH");
									// Update setVals flag.
									valsSet = true;
								}
							}
							break;

						case LINE:
							// If trench mode is selected move to other state.
							if (trenchMode)
							{
								// Deselect line tracking mode.
								NetworkTable->PutBoolean("Line Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::TRENCH_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "LINE");
								// Move to other state.
								selectionState = TRENCH;
							}
							// If fish mode is selected move to other state.
							else if (fishMode)
							{
								// Deselect line tracking mode.
								NetworkTable->PutBoolean("Line Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::FISH_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "LINE");
								// Move to other state.
								selectionState = FISH;
							}
							// If tape mode is selected move to other state.
							else if (tapeMode)
							{
								// Deselect line tracking mode.
								NetworkTable->PutBoolean("Line Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::TAPE_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "LINE");
								// Move to other state.
								selectionState = TAPE;
							}
							else
							{
								// Make sure trench mode is true while in this state.
								NetworkTable->PutBoolean("Line Tracking Mode", true);
								// Only set mode specific values once.
								if (!valsSet)
								{
									// Update networktables values.
									GetJSONValues(NetworkTable, "LINE");
									// Update setVals flag.
									valsSet = true;
								}
							}
							break;

						case FISH:
							// If trench mode is selected move to other state.
							if (trenchMode)
							{
								// Deselect line tracking mode.
								NetworkTable->PutBoolean("Fish Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::TRENCH_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "FISH");
								// Move to other state.
								selectionState = TRENCH;
							}
							// If line mode is selected move to other state.
							else if (lineMode)
							{
								// Deselect trench tracking mode.
								NetworkTable->PutBoolean("Fish Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::LINE_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "FISH");
								// Move to other state.
								selectionState = LINE;
							}
							// If tape mode is selected move to other state.
							else if (tapeMode)
							{
								// Deselect line tracking mode.
								NetworkTable->PutBoolean("Fish Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::TAPE_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "FISH");
								// Move to other state.
								selectionState = TAPE;
							}
							else
							{
								// Make sure trench mode is true while in this state.
								NetworkTable->PutBoolean("Fish Tracking Mode", true);
								// Only set mode specific values once.
								if (!valsSet)
								{
									// Update networktables values.
									GetJSONValues(NetworkTable, "FISH");
									// Update setVals flag.
									valsSet = true;
								}
							}
							break;

						case TAPE:
							// If trench mode is selected move to other state.
							if (trenchMode)
							{
								// Deselect tape tracking mode.
								NetworkTable->PutBoolean("Tape Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::TRENCH_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "TAPE");
								// Move to other state.
								selectionState = TRENCH;
							}
							// If line mode is selected move to other state.
							else if (lineMode)
							{
								// Deselect trench tracking mode.
								NetworkTable->PutBoolean("Tape Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::LINE_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "TAPE");
								// Move to other state.
								selectionState = LINE;
							}
							// If fish mode is selected move to other state.
							else if (fishMode)
							{
								// Deselect line tracking mode.
								NetworkTable->PutBoolean("Tape Tracking Mode", false);
								// Set tracking mode.
								trackingMode = VideoProcess::FISH_TRACKING;
								// Set update values toggle.
								valsSet = false;
								// Store current tackbar values for this tracking state into memory JSON.
								PutJSONValues(NetworkTable, "TAPE");
								// Move to other state.
								selectionState = FISH;
							}
							else
							{
								// Make sure tape mode is true while in this state.
								NetworkTable->PutBoolean("Tape Tracking Mode", true);
								// Only set mode specific values once.
								if (!valsSet)
								{
									// Update networktables values.
									GetJSONValues(NetworkTable, "TAPE");
									// Update setVals flag.
									valsSet = true;
								}
							}
							break;
					}
					takeShapshot = NetworkTable->GetBoolean("Take Shapshot", false);
					enableSolvePNP = NetworkTable->GetBoolean("Enable SolvePNP", false);
					centerLineTolerance = NetworkTable->GetNumber("Center Line Tolerance", 50);
					contourAreaMinLimit = NetworkTable->GetNumber("Contour Area Min Limit", 1211.0);
					contourAreaMaxLimit = NetworkTable->GetNumber("Contour Area Max Limit", 2000);
					trackbarValues[0] = int(NetworkTable->GetNumber("HMN", 1));
					trackbarValues[1] = int(NetworkTable->GetNumber("HMX", 255));
					trackbarValues[2] = int(NetworkTable->GetNumber("SMN", 1));
					trackbarValues[3] = int(NetworkTable->GetNumber("SMX", 255));
					trackbarValues[4] = int(NetworkTable->GetNumber("VMN", 1));
					trackbarValues[5] = int(NetworkTable->GetNumber("VMX", 255));

					// Put NetworkTables data.
					NetworkTable->PutNumber("Target Center X", (targetCenterX + int(NetworkTable->GetNumber("X Setpoint Offset", 0))));
					NetworkTable->PutNumber("Target Width", targetCenterY);
					if (!trackingResults.empty())
					{
						NetworkTable->PutBoolean("Line Is Vertical", trackingResults[0]);
					}
					NetworkTable->PutNumberArray("Tracking Results", trackingResults);
					// NetworkTable->PutNumber("SPNP X Dist", solvePNPValues[0]);
					// NetworkTable->PutNumber("SPNP Y Dist", solvePNPValues[1]);
					// NetworkTable->PutNumber("SPNP Z Dist", solvePNPValues[2]);
					// NetworkTable->PutNumber("SPNP Roll", solvePNPValues[3]);
					// NetworkTable->PutNumber("SPNP Pitch", solvePNPValues[4]);
					// NetworkTable->PutNumber("SPNP Yaw", solvePNPValues[5]);

					// Write current memory JSON document to disk if button is selected.
					if (writeJSON)
					{ 
						// Setup a write data buffer.
						char writeBuffer[65536];
						FileWriteStream writeFileStream(jsonFile, writeBuffer, sizeof(writeBuffer));
						// Create file writer.
						Writer<FileWriteStream> writer(writeFileStream);
						// Give the writer to JSON Document object for data writing.
						visionTuningJSON.Accept(writer);

						// Unselect toggle button after writing is done.
						NetworkTable->PutBoolean("Write JSON", false);
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

		// Close opened file stream.
		fcloseall();

		// Print that program has safely and successfully shutdown.
		cout << "All threads have been released! Program will now stop..." << "\n";
		
		// Kill program.
		return EXIT_SUCCESS;
	}
}