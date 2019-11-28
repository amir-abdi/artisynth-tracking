#pragma once
//Include basic libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <string>
#include <fstream>
#include <time.h>
#include <map>
#include <array>
#include <sys/stat.h>
#include <direct.h>
using namespace std;

//Inclide ND Library Files and Application files 
#include "ndtypes.h"
#include "ndpack.h"
#include "ndopto.h"
#include "certus_aux.h"
//#include "ot_aux.h"

#include "vrpn_server.h"
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"

#include "utils.h"
#include "constants.h"
#include "config.h"

#include <conio.h>

class OptotrakCertus
{
	//constants
	#define SAMPLE_MARKERFREQ	2500.0f		//[1500-4600]
	#define SAMPLE_FRAMEFREQ	100.0f		
	#define SAMPLE_DUTYCYCLE	0.35f		//fraction of time marker is turned on during marker period [0.1 - 0.85]
	#define SAMPLE_VOLTAGE		7.0f		// [7-12] volts is fine
	#define SAMPLE_STREAMDATA	0			//0: send buffered data back on request only, 1: send buffered data back automatically
	
	#define VTK_CERTUS_NTOOLS	1
	
	
public:
	/*bool								Recording;	*/
	OptotrakCertus();
	~OptotrakCertus();
	string								init(int* nmarkers, int* rigidBodyStartMarkers);
	string								close();
	string								collect_data();
	string								GetSdkVersion();
	string								VRPN_server_initiate(const char* str);

	static Transform<double, 3, Affine>		virtual_2_physical(MatrixXd digitized, MatrixXd sphere_centers, Transform<double, 3, Affine>& certusTransformT);
private:
	//VRPN server
	VRPN_Server*						vrpn_server;
	vrpn_Connection_IP*					m_Connection;
	string								send_transformations(Transform<double, 3, Affine>*, uint);
	//markers and rigid bodies
	int									nDevices;
	int									NumberOfRigidBodies;
	int*								nMarkersToActivate;
	Position3d							*p3dData;
	OptotrakRigidStruct					*rigidBodyData;
	string								init_markers_tools(int*);
	string								init_rigid_bodies(int*);
	string								load_rigid_body_from_file(const char *file, int firstMarker);

	//display
	void								display_rigidBodies(uint uFrameNumber, uint uElements, uint uFlags);
	void								display_markers_data(uint uFrameNumber, uint uElements, uint uFlags);
			
	//device handles
	int									PortEnabled[VTK_CERTUS_NTOOLS];  /*! Container used for storing enabled tools */
	int									PortHandle[VTK_CERTUS_NTOOLS];   /*! Container used for storing tool handles */		
	int									nCurMarker;
	OptotrakSettings					dtSettings;
	ApplicationDeviceInformation 		*pdtDevices;
	DeviceHandle 						*pdtDeviceHandles;
	DeviceHandleInfo 					*pdtDeviceHandlesInfo;

	//transformations	
	MatrixXd							lower_digitized;
	MatrixXd							upper_digitized;
	MatrixXd							sphere_centers_lower;
	MatrixXd							sphere_centers_upper;
	//Transform<double, 3, Affine>		lower_final; //tried to use this for transforming to local coordinates
	
	string								set_FOR(int rigidBodyID);
	
	//optotrak error
	char								szNDErrorString[MAX_ERROR_STRING_LENGTH + 1];
	string								get_error();

	//optotrak utils
	void								sleep(unsigned int uSec); //optotrak sleep
	
	//initialize
	string								init_optotrak();
	string								init_devices(int* nMarkers);
	string								activate_device(int stroberID, int num_markers);	
	string								set_optional_flags();
	string								load_camera_parameters();
	string								init_data_collection();			
	string								read_landmark_files();
	string								init_write_tracking();

	//end
	string								StopRecording();
	string								DisableToolPorts();
	string								EnableToolPorts();
	
	//digitizing
	string								digitize_rigid_body(int rigidID);
	string								digitize();
	int									stylusID;
	string								digitizedFileNames[3];	


	//Write tracking
	string								write_tracking(Transform<double, 3, Affine>*, double, string);
	//string								finalize_tracking_files();
	string								finalize_tracking_file(string);
	ofstream							ofs_track_trans_upper;
	ofstream							ofs_track_trans_lower;
	ofstream							ofs_track_raw_upper;
	ofstream							ofs_track_raw_lower;
	string								trans_file_upper;
	string								trans_file_lower;
	string								raw_file_upper;
	string								raw_file_lower;
	//clock_t								recording_start_time;
	void								close_finalize_tracking_files();
	double								last_recorded_time;
	double								first_recorded_time;
};

