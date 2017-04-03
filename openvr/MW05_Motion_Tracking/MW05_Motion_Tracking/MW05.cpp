/* [MW05] VR Harpoon - Motion Control v1.0
 * 
 * PURPOSE: 
 * - Control XY gantry according to position and orientation of VR headset.
 * 
 * DEPENDENCIES: 
 * - OpenVR
 *
 * INSTRUCTIONS TO CALIBRATE GANTRY:
 * - If you run room calibration, you will likely need to re-sync the axes of the gantry with the HTC Vive.
 * - 1. Open SteamVR -> Settings -> Developer Tab
 * - 2. Find the "Room and Tracking" heading and select "Room Overview"
 * - 3. Compare the axes in the "Room Overview" window to the real-world room layout. 
 * - 4. Change global variable ROTATE_AXES_CW to the degrees required to rotate the "Room Overview" axes such that it matches the real-world room layout.
 * - 5. Re-run the program to confirm the new axes are correct.
 *
 * COMMENTS:
 * - Uses absolute positioning
   - Coordinate system (the '|0|' are motors)

		 |0|  x ^
				|
				|
				|
		 |0|    ---------> z
	 
 * AUTHOR: Justin Lam
 * LAST EDITED: December 19, 2016
 */

#include <iostream>
#include <string>
#include <ctgmath>
#include <iomanip>
#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <stdlib.h>
#include <time.h>
#include "openvr.h"
#include "SerialClass.h"
#include "IniWriter.h"
#include "IniReader.h"

// COM Port to communicate with Teensy 
//#define COM_PORT "\\\\.\\COM60"		// in Device Manager, should be under PORTS (COM & LPT) -> "Teensy USB Serial"
//#define COM_PORT "COM60"		// in Device Manager, should be under PORTS (COM & LPT) -> "Teensy USB Serial"
//
//// Global variables for user interaction
//#define ROTATE_AXES_CW 180			// degrees to rotate axes to align gantry w/ HMD
//#define VR_TIME_INTERVAL 2			// frequency to pull poses off the HTC vive (lower is more better but uses more memory)
//#define STARTUP_DELAY 3				// number of seconds at startup to give user time to positioxn themselve in center of gantry
//#define TIME_TO_NEXT_POSE 10		// 0 means instant, otherwise prediction of future pose in X seconds
//#define POSE_THRESHOLD 0.025			// in meters. Required positional change before servant head moves
//#define DIST_FROM_USER 0.3			// fixed distance for the servant head to stay behind the user, in meters
//
//// Global variables for the gantry and communication w/ microcontroller
//#define PULLEY_RADIUS 0.02			// radius in m <- THIS DETERMINES THE DISTANCE OF MEASURE
//#define STEP_NUM 0.5				// stepper param: 1/2 step    
//#define STEP_SIZE 1.8				// stepper param: degrees
//#define CALIBRATION_CODE 0xF1ACA	// this needs to match on the arduino
//#define STOP_CODE 0xEF355			// also needs to match arduino

bool sendData = false;
uint8_t encoded_data[12]; // NEEDS TO BE CHANGED IF CHANGING STRUCT SIZE
vr::HmdVector3_t zerod_orientation;

char settingsFile[] = "C:/Users/Vlad/Documents/Bit Bucket/MW05/VR-Gantry-Software/openvr/MW05_Motion_Tracking/MW05_Motion_Tracking/UserSettings.ini";
char settingsSection1[] = "Setting";
char settingsSection2[] = "Microcontroller";

CIniWriter iniWriter(settingsFile);
CIniReader iniReader(settingsFile);

char *COM_PORT;
static volatile int ROTATE_AXES_CW;
static volatile int VR_TIME_INTERVAL;
static volatile int STARTUP_DELAY;
static volatile int TIME_TO_NEXT_POSE;
static volatile float POSE_THRESHOLD;
static volatile float DIST_FROM_USER;
static volatile bool FIRST_ROOM_SETUP;
static volatile float ORIGIN_X;
static volatile float ORIGIN_Y;
static volatile float ORIGIN_Z;

static volatile float PULLEY_RADIUS;
static volatile float STEP_NUM;
static volatile float STEP_SIZE;
static volatile int CALIBRATION_CODE;
static volatile int STOP_CODE;

// Store values in UserSettings.ini; this function doesn't actually get called but is here in case the settings file gets deleted.
void writeUserSettings(void) {
	iniWriter.WriteString (settingsSection1, "SerialPort",			 "COM60");		// in Device Manager, should be under PORTS (COM & LPT) -> "Teensy USB Serial"
	iniWriter.WriteInteger(settingsSection1, "GantryAxisRotation",	 180);			// degrees to rotate axes to align gantry w/ HMD
	iniWriter.WriteInteger(settingsSection1, "SamplingFrequency",	 2);			// frequency to pull poses off the HTC vive (lower is more better but uses more memory)
	iniWriter.WriteInteger(settingsSection1, "StartupDelay",		 3);			// number of seconds at startup to give user time to positioxn themselve in center of gantry
	iniWriter.WriteInteger(settingsSection1, "TimeToNextPose",		 10);			// 0 means instant, otherwise prediction of future pose in X seconds
	iniWriter.WriteFloat  (settingsSection1, "PoseChangeThreshold",  0.025f);		// in meters. Required positional change before servant head moves
	iniWriter.WriteFloat  (settingsSection1, "DistanceFromUser",	 0.3f);			// fixed distance for the servant head to stay behind the user, in meters
	iniWriter.WriteBoolean(settingsSection1, "FirstRoomSetup",		 false);		// Only need to zero the origin when room setup is changed
	iniWriter.WriteFloat  (settingsSection1, "OriginX",				 0);
	iniWriter.WriteFloat  (settingsSection1, "OriginY",				 0);
	iniWriter.WriteFloat  (settingsSection1, "OriginZ",				 0);

	iniWriter.WriteFloat  (settingsSection2, "PulleyRadius",	0.02);				// radius in m <- THIS DETERMINES THE DISTANCE OF MEASURE
	iniWriter.WriteFloat  (settingsSection2, "StepNum",		    0.5);				// stepper param: 1/2 step    
	iniWriter.WriteFloat  (settingsSection2, "StepSize",		1.8);				// stepper param: degrees
	iniWriter.WriteInteger(settingsSection2, "CalibrationCode", 0xF1ACA);			// this needs to match on the arduino
	iniWriter.WriteInteger(settingsSection2, "StopCode",		0xEF355);			// also needs to match arduino
}

// Import values from UserSettings.ini.
void readUserSettings() {
	COM_PORT =			iniReader.ReadString (settingsSection1, "SerialPort", "");		
	ROTATE_AXES_CW =	iniReader.ReadInteger(settingsSection1, "GantryAxisRotation", 0);
	VR_TIME_INTERVAL =	iniReader.ReadInteger(settingsSection1, "SamplingFrequency", 0);
	STARTUP_DELAY =		iniReader.ReadInteger(settingsSection1, "StartupDelay",	0);
	TIME_TO_NEXT_POSE = iniReader.ReadInteger(settingsSection1, "TimeToNextPose", 0);
	POSE_THRESHOLD =	iniReader.ReadFloat  (settingsSection1, "PoseChangeThreshold", 0);
	DIST_FROM_USER =	iniReader.ReadFloat  (settingsSection1, "DistanceFromUser",	0);
	FIRST_ROOM_SETUP =  iniReader.ReadBoolean(settingsSection1, "FirstRoomSetup", false);
	ORIGIN_X =			iniReader.ReadFloat	 (settingsSection1, "OriginX", 0);
	ORIGIN_Y =			iniReader.ReadFloat	 (settingsSection1, "OriginY", 0);
	ORIGIN_Z =			iniReader.ReadFloat	 (settingsSection1, "OriginZ", 0);

	PULLEY_RADIUS =		iniReader.ReadFloat  (settingsSection2, "PulleyRadius",	0);
	STEP_NUM =			iniReader.ReadFloat  (settingsSection2, "StepNum", 0);
	STEP_SIZE =			iniReader.ReadFloat  (settingsSection2, "StepSize",	0);
	CALIBRATION_CODE =	iniReader.ReadInteger(settingsSection2, "CalibrationCode", 0);
	STOP_CODE =			iniReader.ReadInteger(settingsSection2, "StopCode",	0);
}

// Pose data from VR headset
struct TrackingData {
	float x, y, z, 
		phi, theta, psi,
		dx, dy, dz;
};

TrackingData origin;
TrackingData zerod_pose;

// Position and checksum data to send to Arduino
typedef struct ARDUINO_CMDS_S {
	int32_t x_pos;
	int32_t y_pos;
	uint16_t check_sum;
} arduino_cmd_t;

arduino_cmd_t arduino_cmd;

// Positional data
struct TrackingDataXY {
	float x = 0, z = 0;
};

// Wrap 2D array in a struct for easier passing
struct Matrix33_t
{
	float m[3][3];
};

BOOL CtrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// CTRL-CLOSE: confirm that the user wants to exit. 
	case CTRL_CLOSE_EVENT:

		//printf("Saving user settings.");
		//writeUserSettings();
		return(TRUE);

	default:
		return FALSE;
	}
}

void modifyUserSettings() {
	int user_input;
	char tempString[128];
	int tempInt;
	float tempFloat;

	std::cout << "\nSelect one of the variables to edit.\n";
	std::cout << "   1 - Serial port: " << COM_PORT << std::endl;
	std::cout << "   2 - Gantry axis rotation: " << ROTATE_AXES_CW << " degrees" << std::endl;
	std::cout << "   3 - Sampling frequency: " << VR_TIME_INTERVAL << " ms" << std::endl;
	std::cout << "   4 - Startup delay: " << STARTUP_DELAY << " ms" << std::endl;
	std::cout << "   5 - Time to next pose: " << TIME_TO_NEXT_POSE << " ms" << std::endl;
	std::cout << "   6 - Pose change threshold: " << POSE_THRESHOLD << " meters" << std::endl;
	std::cout << "   7 - Distance behind user: " << DIST_FROM_USER << " meters" << std::endl;
	std::cout << "   8 - Origin coordinates (x, z): (" << ORIGIN_X << ", " << ORIGIN_Z << ")\n";
	std::cout << "   0 - Return to menu\n";

	std::cout << "Option: ";
	std::cin >> user_input;

	switch (user_input) {
	case 1:
		std::cout << "Enter new COM port name (ex. 'COM60'): ";
		std::cin >> tempString;
		iniWriter.WriteString(settingsSection1, "SerialPort", tempString);
		break;
	case 2:
		std::cout << "Enter new rotation value, + for CW or - for CCW (ex. '90' or '-90'): ";
		std::cin >> tempInt;
		iniWriter.WriteInteger(settingsSection1, "GantryAxisRotation", tempInt);
		break;
	case 3:
		std::cout << "Enter new sampling frequency (ex. '3' means 'every 3 milliseconds'): ";
		std::cin >> tempInt;
		iniWriter.WriteInteger(settingsSection1, "SamplingFrequency", tempInt);
		break;
	case 4:
		std::cout << "Enter new startup delay (ex. '3' means 'starts in 3 seconds'): ";
		std::cin >> tempInt;
		iniWriter.WriteInteger(settingsSection1, "StartupDelay", tempInt);
		break;
	case 5:
		std::cout << "Enter new time to next pose: ";
		std::cin >> tempInt;
		iniWriter.WriteInteger(settingsSection1, "TimeToNextPose", tempInt);
		break;
	case 6:
		std::cout << "Enter new pose change threshold (ex. '0.05', in meters): ";
		std::cin >> tempFloat;
		iniWriter.WriteFloat(settingsSection1, "PoseChangeThreshold", tempFloat);
		break;
	case 7:
		std::cout << "Enter new distance behind user (ex. '0.025', in meters): ";
		std::cin >> tempFloat;
		iniWriter.WriteFloat(settingsSection1, "DistanceFromUser", tempFloat);
		break;
	default:
		std::cout << "That was not a valid option: " << user_input;
		break;
	}
	readUserSettings();
	std::cout << "Settings changed! Tracking continued.\n";
}

void loadStartup() {
	int user_input;

	std::cout << ".___  ___.  __       _______.___________.____    ____ ____    __    ____  _______     _______.___________.\r\n";
	std::cout << "|   \\/   | |  |     /       |           |\\   \\  /   / \\   \\  /  \\  /   / |   ____|   /       |           |\r\n";
	std::cout << "|  \\  /  | |  |    |   (----`---|  |----` \\   \\/   /   \\   \\/    \\/   /  |  |__     |   (----`---|  |----`\r\n";
	std::cout << "|  |\\/|  | |  |     \\   \\       |  |       \\_    _/     \\            /   |   __|     \\   \\       |  |     \r\n";
	std::cout << "|  |  |  | |  | .----)   |      |  |         |  |        \\    /\\    /    |  |____.----)   |      |  |     \r\n";
	std::cout << "|__|  |__| |__| |_______/       |__|         |__|         \\__/  \\__/     |_______|_______/       |__|     \r\n";

	std::cout << "\n                             *** Mistywest VR Gantry Tracking V1.0 ***\n\n";
	std::cout << "                          Hit the ESC key at any point to stop the gantry.\n\n";

	// Startup functions
	readUserSettings();		// import global variables from UserSettings.ini
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);		// initialize handler to save user settings on console exit

	std::cout << "\nSelect one of the options below followed by the ENTER key.\n";
	std::cout << "   1 - Start the gantry!\n";
	std::cout << "   2 - Modify user settings\n";

	std::cout << "Option: ";
	std::cin >> user_input;

	if (user_input == 2) {
		modifyUserSettings();
	}
}

// End the tracking
void over()
{
	std::cout << "\nPress the any key to continue...";
	getchar();
}

// Check for vr startup errors
int checkVrSetup(int init)
{
	if (init == 0)
		std::cout << "SteamVR connected! \n";
	else
	{
		std::cout << "SteamVR not connected. Error level = " << init << std::endl;
		//over();
	}
	return(init);
}

// Set zerod poses on startup
void setZerodPositions(TrackingData currentPose)
{
	origin.x = currentPose.x;
	origin.z = currentPose.z;

	iniWriter.WriteFloat(settingsSection1, "OriginX", currentPose.x);
	iniWriter.WriteFloat(settingsSection1, "OriginZ", currentPose.z);

	// Change this vector if the direction is switched
	//zerod_orientation.v[0] = 0;		// x
	//zerod_orientation.v[1] = 0;		// y - vertical, so this should always be zero
	//zerod_orientation.v[2] = DIST_FROM_USER;	// z
}

// Extract the quaternion from the pose matrix
vr::HmdQuaternion_t getQuaternion(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}

// Calculates the rotation matrix from the HMD matrix
Matrix33_t HmdToRotationMatrix(vr::HmdMatrix34_t matrix_hmd)
{
	Matrix33_t rot;
	vr::HmdQuaternion_t q = getQuaternion(matrix_hmd);
	
	rot.m[0][0] = 1 - 2 * q.y*q.y - 2 * q.z*q.z;
	rot.m[0][1] = 2 * q.x*q.y - 2 * q.z*q.w;
	rot.m[0][2] = 2 * q.x*q.z + 2 * q.y*q.w;
	rot.m[1][0] = 2 * q.x*q.y + 2 * q.z*q.w;
	rot.m[1][1] = 1 - 2 * q.x*q.x - 2 * q.z*q.z;
	rot.m[1][2] = 2 * q.y*q.z - 2 * q.x*q.w;
	rot.m[2][0] = 2 * q.x*q.z - 2 * q.y*q.w;
	rot.m[2][1] = 2 * q.y*q.z + 2 * q.x*q.w;
	rot.m[2][2] = 1 - 2 * q.x*q.x - 2 * q.y*q.y;

	return rot;
}

// Multiplication of a 3x3 matrix by a 3x1 vector
vr::HmdVector3_t MatrixProduct31(Matrix33_t rotation_m, vr::HmdVector3_t position)
{
	vr::HmdVector3_t result;

	result.v[0] = 0;
	result.v[1] = 0;
	result.v[2] = 0;

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			result.v[row] += rotation_m.m[row][col] * position.v[col];
		}
	}

	return result;
}

// Get relevant tracking data (cartesian rotation and position)
TrackingData getTrackingData(vr::HmdMatrix34_t matrix, bool reset_gantry)
{
	TrackingData trackingData;
	vr::HmdVector3_t rotated_position;
	Matrix33_t rotation_m;

	float pos1, pos2;

	rotation_m = HmdToRotationMatrix(matrix);
	rotated_position = MatrixProduct31(rotation_m, zerod_orientation);

	if (reset_gantry) {
		trackingData.y = matrix.m[1][3];
	}
	else {
		trackingData.y = matrix.m[1][3] + rotated_position.v[1];
	}

	pos2 = matrix.m[0][3] + rotated_position.v[0];
	pos1 = matrix.m[2][3] + rotated_position.v[2];

	if (ROTATE_AXES_CW == 270 || ROTATE_AXES_CW == -90) {
		trackingData.x = pos2;
		trackingData.z = -pos1;
	}
	else if (ROTATE_AXES_CW == 180) {
		trackingData.x = -pos1;
		trackingData.z = -pos2;
	}
	else if (ROTATE_AXES_CW == 90 || ROTATE_AXES_CW == -270) {
		trackingData.x = -pos2;
		trackingData.z = pos1;
	}
	else {
		trackingData.x = pos1;
		trackingData.z = pos2;
	}

	 return trackingData;
}

 // Show rotation angles and position data on console for debugging
 void displayTrackingData(TrackingData data)
 {
	 std::cout << std::fixed;
	 std::cout << std::setprecision(2);

	 std::cout << "Phi: " << data.phi << "   ";
	 std::cout << "Theta: " << data.theta << "   ";
	 std::cout << "Psi: " << data.psi << "   ";
	 std::cout << "X: " << data.x << "   ";
	 std::cout << "Y: " << data.y << "   ";
	 std::cout << "Z: " << data.z << "\t\r";
 }

 // Show raw pose matrix on console for debugging
 void displayRawPose(vr::HmdMatrix34_t position)
 {
	 std::cout.precision(2);

	 std::cout << position.m[0][0] << ", ";
	 std::cout << position.m[1][0] << ", ";
	 std::cout << position.m[2][0] << "\t";

	 std::cout << position.m[0][1] << ", ";
	 std::cout << position.m[1][1] << ", ";
	 std::cout << position.m[2][1] << "\t";

	 std::cout << position.m[0][2] << ", ";
	 std::cout << position.m[1][2] << ", ";
	 std::cout << position.m[2][2] << "\t";

	 std::cout << position.m[0][3] << ", ";
	 std::cout << position.m[1][3] << ", ";
	 std::cout << position.m[2][3] << "\n" << std::flush;
 }

 // Calculate the zeroed pose based on the initial origin calibration on boot
 TrackingData zeroPose(TrackingData current) 
 {
	 TrackingData diff;

	 diff.x = current.x - origin.x;
	 diff.y = current.y - origin.y;
	 diff.z = current.z - origin.z;
	 diff.phi = current.phi - origin.phi;
	 diff.psi = current.psi - origin.psi;
	 diff.theta = current.theta - origin.theta;

	 return diff;
 }

 // Calculate difference between current and previous pose
 TrackingData deltaPose(TrackingData current, TrackingData prev_pose)
 {
	TrackingData delta;

	delta.x = current.x - prev_pose.x;
	delta.y = current.y - prev_pose.y;
	delta.z = current.z - prev_pose.z;
	delta.phi = current.phi - prev_pose.phi;
	delta.psi = current.psi - prev_pose.psi;
	delta.theta = current.theta - prev_pose.theta;

	return delta;
 }
 
 // Parse the raw pose and send through serial. 
 void parsePose(TrackingData currentPose)
 {
	 static TrackingData prev_pose;

	 TrackingData change_in_pose;
	 bool checkX = false, checkZ = false, checkBounds = false;
	 
	 // Determine the pose relative to the origin
	 zerod_pose = zeroPose(currentPose);

	 // Calculate the change in current pose from previous
	 change_in_pose = deltaPose(zerod_pose, prev_pose);
	 
	 //if (first_pass) {
		// printf("Change - X: %5.2f, Z: %5.2f\r", change_in_pose.x, change_in_pose.z);
	 //}
	 
	 // Check pose change conditions before sending commmand to serial
	 checkX = abs(change_in_pose.x) > POSE_THRESHOLD;
	 checkZ = abs(change_in_pose.z) > POSE_THRESHOLD;

	 // Send data if conditions are met
	 if (checkX || checkZ) {
		 prev_pose = zerod_pose;
		 sendData = true;
	 } 
 }
 
 // Fletcher 16 checksum to prevent garbled data through serial
uint16_t Fletcher16(uint8_t *data, int count)
{
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;
	int index;
	 
	for (index = 0; index < count; ++index) {
		sum1 = (sum1 + data[index]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}
	 
	return (sum2 << 8) | sum1;
}

#define FinishBlock(X) (*code_ptr = (X), code_ptr = dst++, code = 0x01)	// for COBS encoding

// Stuff data using COBS encoding
void StuffData(const uint8_t *ptr, unsigned long length, uint8_t *dst)
{
	const uint8_t *end = ptr + length;
	uint8_t *code_ptr = dst++;
	uint8_t code = 0x01;

	while (ptr < end)
	{
		if (*ptr == 0)
			FinishBlock(code);
		else
		{
			*dst++ = *ptr;
			if (++code == 0xFF)
				FinishBlock(code);
		}
		ptr++;
	}

	FinishBlock(code);
}

// Unstuff data using COBS encoding
void UnStuffData(const uint8_t *ptr, unsigned long length, uint8_t *dst)
{
	const uint8_t *end = ptr + length;
	while (ptr < end)
	{
		int i, code = *ptr++;
		for (i = 1; i<code; i++)
			*dst++ = *ptr++;
		if (code < 0xFF)
			*dst++ = 0;
	}
}

// Convert linear distance in meters to motor steps
int32_t convertDistToSteps(float meters)
{
	return meters * 360 / (2 * 3.141592654 * PULLEY_RADIUS * STEP_SIZE * STEP_NUM);
}

// Send pose data through serial communication
void sendPoseToSerial(Serial *SP)
{
	char incomingData[6] = ""; 
	int dataLength = 5;	
	int readResult = 0;
	bool dataSent = false;

	arduino_cmd.x_pos = convertDistToSteps(zerod_pose.x);
	arduino_cmd.y_pos = convertDistToSteps(zerod_pose.z);
	arduino_cmd.check_sum = Fletcher16((uint8_t *)(&arduino_cmd), 8);

	StuffData((uint8_t *)(&arduino_cmd), sizeof(arduino_cmd), &encoded_data[1]);
	encoded_data[0] = 0;
	//packArduinoCmd(zerod_pose.x, zerod_pose.y);
	dataSent = SP->WriteData((char *)(encoded_data), sizeof(arduino_cmd) + 2);
	printf("X: %5.2d, Z: %5.2d\r", arduino_cmd.x_pos, arduino_cmd.y_pos);

	if (!dataSent) {
		std::cout << "Packet not sent!" << std::endl;
	}
}

// Send designated command to arduino to run calibration procedure
void runGantryCalibration(Serial *SP)
{
	// Arbitrary command to run calibration on Arduino
	arduino_cmd.x_pos = CALIBRATION_CODE;
	arduino_cmd.y_pos = CALIBRATION_CODE;
	arduino_cmd.check_sum = Fletcher16((uint8_t *)(&arduino_cmd), 8);

	StuffData((uint8_t *)(&arduino_cmd), sizeof(arduino_cmd), &encoded_data[1]);
	encoded_data[0] = 0;
	SP->WriteData((char *)(encoded_data), sizeof(arduino_cmd) + 2);
}

// Send designated command to arduino to stop motors
void stopMotors(Serial *SP)
{
	// Arbitrary command to run calibration on Arduino
	arduino_cmd.x_pos = STOP_CODE;
	arduino_cmd.y_pos = STOP_CODE;
	arduino_cmd.check_sum = Fletcher16((uint8_t *)(&arduino_cmd), 8);

	StuffData((uint8_t *)(&arduino_cmd), sizeof(arduino_cmd), &encoded_data[1]);
	encoded_data[0] = 0;
	SP->WriteData((char *)(encoded_data), sizeof(arduino_cmd) + 2);
}

// Blocking timer to give user some time to put on headset and stand below the sherpa head before zeroing the position
void runStartupTimer() {
	int countDown = STARTUP_DELAY;	// seconds

	for (int i = countDown; i > 0; i--) {
		std::cout << "Starting in " << i << "...\r";
		Sleep(1000);
	}
}

int _tmain(int argc, _TCHAR* argv[])
 {
	// Initialize vr classes
	vr::EVRInitError init;
	vr::EVRApplicationType appType;
	vr::ETrackingUniverseOrigin universeOrigin = vr::TrackingUniverseStanding; // Poses are provided relative to the seated zero pose
	vr::TrackedDevicePose_t outputData;
	vr::HmdMatrix34_t position;

	TrackingData currentPose, previousPose;// changeInPose; // currentPose,
	
	//bool firstPass = true;
	bool resetGantry = true;
	//bool pauseGantry = false;

	clock_t t_curr;
	clock_t tvr_prev = 0;

	loadStartup();

	// Construct the COM port name
	char portName[16];
	strcpy(portName, "\\\\.\\");	// need this due to windows (see Serial.h for details)
	strcat(portName, COM_PORT);

	int user_input = 1;
	
	// Check SteamVR setup
	std::cout << "-> Connecting to SteamVR... ";
	do {
		appType = vr::VRApplication_Background;
		vr::VR_Init(&init, appType);

		if (init) {
			if (init == 121) {
				std::cout << "\nPlease open the SteamVR application and try again. ";
			}
			else {
				std::cout << "\nError with SteamVR setup. ";
			}
						
			std::cout << "Select one of the options below followed by the ENTER key.\n";
			std::cout << "   1 - Retry connection with SteamVR\n";
			std::cout << "   2 - Continue without SteamVR\n";
			std::cout << "   3 - Exit\n";

			std::cout << "\nOption: ";
			std::cin >> user_input;

			while (user_input < 1 || user_input > 3) {
				std::cout << "Please enter a valid option: ";
				std::cin >> user_input;
			}
		}
		else {
			break;	// continue if successfully connected
		}

		if (user_input == 3) {
			return 0;	// exit program
		}

	} while (user_input == 1);

	std::cout << "SteamVR connected!\n";

	// Check serial setup
	std::cout << "-> Connecting to serial port " << COM_PORT << "... ";
	Serial* SP = new Serial(portName);

	while (!SP->IsConnected()) 		{
		delete SP;	// delete previous member before creating a new one

		std::cout << "\nSerial not initialized. Try replugging USB. Select one of the options below followed by the ENTER key.\n";
		std::cout << "   1 - Retry serial connection\n";
		std::cout << "   2 - Modify settings\n";
		std::cout << "   3 - Exit\n";

		std::cout << "\nOption: ";
		std::cin >> user_input;
	
		// Retry serial connection
		if (user_input == 1) {
			std::cout << "-> Connecting to serial port " << COM_PORT << "... ";
					
			Serial* SP = new Serial(portName);
		}
		else if (user_input == 2) {
			modifyUserSettings();
		}
		// Exit program
		else if (user_input == 3) {
			return 0;	
		}

	}
	std::cout << "Serial port connected!\n";
	
	std::cout << "Setup complete. Select one of the options below followed by the ENTER key.\n";
	std::cout << "   1 - Play area is unchanged. Start the gantry.\n";
	std::cout << "   2 - Play area has changed and origin needs to be re-zeroed.\n";

	std::cout << "\nOption: ";
	std::cin >> user_input;
	getchar();

	while (user_input < 1 || user_input > 2 ) {
		std::cout << "\nPlease enter a valid option: ";
		std::cin >> user_input;
		getchar();
	}

	if (user_input == 1) {
		origin.x = ORIGIN_X;
		origin.z = ORIGIN_Z;
	}
	else if (user_input == 2) {
		FIRST_ROOM_SETUP = true;
	}

	//if (user_input == 1) {
	std::cout << "Calibrating gantry axes... Press ENTER when complete and to begin tracking.\n";
	runGantryCalibration(SP);
	getchar();

	runStartupTimer();
	//}

	//vr::VRSystem()->GetControllerState(0, )

	std::cout << "Let's get tracking!\n";

	// Set the orientation vector
	zerod_orientation.v[0] = 0;		// x
	zerod_orientation.v[1] = 0;		// y - vertical, so this should always be zero
	zerod_orientation.v[2] = DIST_FROM_USER;		// z
		
	while (1)	{
		t_curr = clock();

		// Get VR poses
		if (t_curr - tvr_prev >= VR_TIME_INTERVAL)	{
			vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(universeOrigin, TIME_TO_NEXT_POSE, &outputData, 1);	// can change the second parameter but system responsiveness is ultimately determined by the physical system and how quickly it can accelerate
			position = outputData.mDeviceToAbsoluteTracking;
			tvr_prev = t_curr;

			if (outputData.bPoseIsValid) {
				currentPose = getTrackingData(position, resetGantry);

				// Set the origin of the play space if it's the first time room setup is run
				if (FIRST_ROOM_SETUP) {
					setZerodPositions(currentPose);
					//firstPass = false;
					FIRST_ROOM_SETUP = false;
					iniWriter.WriteBoolean(settingsSection1, "FirstRoomSetup", FIRST_ROOM_SETUP);
				}

				// If the gantry (ie. stepper motors) need to be reset, then need to reinitialize the orientation vector
				if (resetGantry) {
					//zerod_orientation.v[2] = DIST_FROM_USER;
					resetGantry = false;
				}

				parsePose(currentPose);
				//displayTrackingData(currentPose);
				//displayRawPose(position);
			}
		}

		// Send poses to serial
		if (sendData) {
			sendPoseToSerial(SP);
			sendData = false;
		}

		// Check if user needs to stop tracking
		if (GetAsyncKeyState(VK_ESCAPE)) {
			stopMotors(SP);
			//resetGantry = false;

			std::cout << "\n--PAUSED--\n";
			std::cout << "Select one of the options below followed by the ENTER key.\n";
			std::cout << "   1 - Continue tracking\n";
			std::cout << "   2 - Recalibrate\n";
			std::cout << "   3 - Modify settings\n";
			std::cout << "   4 - Exit tracking\n";

			std::cout << "\nOption: ";
			std::cin >> user_input;

			// Recalibrate gantry
			if (user_input == 2) {
				getchar();		// read the ENTER key otherwise it won't wait until calibration is complete and user is ready
				std::cout << "Calibrating gantry axes... Press ENTER when complete and to begin tracking.\n";
				runGantryCalibration(SP);
				getchar();

				//runStartupTimer();
				
				//firstPass = true;

				// Reset the orientation vector to 0 before reinitializing 
				//zerod_orientation.v[2] = 0;		// will be set to DIST_FROM_USER
				resetGantry = true;
				
			}
			else if (user_input == 3) {
				modifyUserSettings();
			}
			// Exit tracking loop
			else if (user_input == 4) {
				break;
			}
		}
		
	}
	over();
	return(0);

}