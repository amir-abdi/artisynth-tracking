#include "OptotrakCertus.h"
//Inclide ND Library Files and Application files 
#include "certus_aux.c"
//#include "ot_aux.c"

OptotrakCertus::OptotrakCertus()
{		
	pdtDevices = NULL;
	pdtDeviceHandles = NULL;
	pdtDeviceHandlesInfo = NULL;
	p3dData = NULL;
	dtSettings.nMarkers = 0;
	dtSettings.fFrameFrequency = SAMPLE_FRAMEFREQ;
	dtSettings.fMarkerFrequency = SAMPLE_MARKERFREQ;
	dtSettings.nThreshold = 30;
	dtSettings.nMinimumGain = 160;
	dtSettings.nStreamData = SAMPLE_STREAMDATA;
	dtSettings.fDutyCycle = SAMPLE_DUTYCYCLE;
	dtSettings.fVoltage = SAMPLE_VOLTAGE;
	dtSettings.fCollectionTime = 1.0;
	dtSettings.fPreTriggerTime = 0.0;
	nDevices = 0;
	//memset(nMarkersToActivate, 0, 3 * sizeof(int));
	nMarkersToActivate = NULL;	

	for (int i = 0; i < VTK_CERTUS_NTOOLS; ++i) {
		PortHandle[i] = 0;
		PortEnabled[i] = 0;
	}
	//Recording = false;
	//this->logFile = "";

	cout << "Logging into " << get_log_file() << endl;

	digitizedFileNames[1] = Config::config["digitize_upper"]; //string("../digitized_points/") + to_string(1) + string("digitized_points.xyz");
	digitizedFileNames[2] = Config::config["digitize_lower"]; //string("../digitized_points/") + to_string(2) + string("digitized_points.xyz");
	
	stylusID = -1;
	
}

OptotrakCertus::~OptotrakCertus()
{
	//if (this->Recording)
	StopRecording();
}

string	OptotrakCertus::init(int* nmarkers, int* rigidBodyStartMarkers)
{
	string err;
	/*int nmarkers[] = {4,12,10};*/
	if ((err = init_optotrak()) != "no_error")
		return err;
	if ((err = init_devices(nmarkers)) != "no_error")
		return err;
	//set_settings();
	ApplicationDetermineCollectionParameters(nDevices, pdtDevices, &dtSettings);
	set_optional_flags();
	if ((err = load_camera_parameters()) != "no_error")
		return err;
	if ((err = init_markers_tools(rigidBodyStartMarkers)) != NO_ERROR_STR)
		return err;
	if (Config::digitize_flag && ((err = digitize()) != NO_ERROR_STR))
		return err;
	if (Config::digitize_flag)
		return string("Digitization sucessfull. You now need to restart the test with digitize flag set to false.");
	if ((err = read_landmark_files()) != NO_ERROR_STR)
		return err;
	if ((err = set_FOR(Config::FORBody_index)) != NO_ERROR_STR) // set FOR to mandible
		return err;
	
	return NO_ERROR_STR;
}

string OptotrakCertus::init_write_tracking()
{
	if (Config::write_tranformations_flag)
	{
		trans_file_upper = get_tracking_file("upper", "trans");
		trans_file_lower = get_tracking_file("lower", "trans");
		try 
		{
			ofs_track_trans_upper.open((trans_file_upper).c_str());			
			ofs_track_trans_upper << "linear 7 explicit\n";

			ofs_track_trans_lower.open((trans_file_lower).c_str());
			ofs_track_trans_lower << "linear 7 explicit\n";
		}
		catch (exception e)
		{
			return string("Error in init_write_tracking, tracking_trans file: ") + string(e.what());
		}
	}

	if (Config::write_raw_flag)
	{
		raw_file_upper = get_tracking_file("upper", "raw");
		raw_file_lower = get_tracking_file("lower", "raw");

		try
		{
			ofs_track_raw_upper.open((raw_file_upper).c_str());
			ofs_track_raw_lower.open((raw_file_lower).c_str());

			ofs_track_raw_upper << "linear 7 explicit\n";
			ofs_track_raw_lower << "linear 7 explicit\n";

		}
		catch (exception e)
		{
			return string("Error in init_write_tracking, tracking raw file: ") + string(e.what());
		}
	}

	return NO_ERROR_STR;
}

string OptotrakCertus::init_markers_tools(int* rigidBodyStartMarkers)
{
	string err;
	
	//if ((err = EnableToolPorts()) != NO_ERROR_STR)
		//return err;
	if ((err = init_rigid_bodies(rigidBodyStartMarkers)) != NO_ERROR_STR)
		return err;
	if ((err = init_data_collection()) != NO_ERROR_STR)
		return err;

	fprintf(stdout, "*********************************************************.\n");
	LOG_DEBUG(string("Optotrak Certus initiazed with ") + to_string(nDevices) + string("strobers"));
	for (int i = 0; i<nDevices; ++i)
		LOG_DEBUG(to_string(nMarkersToActivate[i]) + string(" markers on strober ") + to_string( i));
	fprintf(stdout, "*********************************************************.\n");

	rigidBodyData = new OptotrakRigidStruct[this->NumberOfRigidBodies];
	return NO_ERROR_STR;	
}

string OptotrakCertus::init_optotrak()
{
	/* Load the system of processors. 	*/
	LOG_DEBUG("TransputerLoadSystem...");
	if (TransputerLoadSystem("system") != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	}
	sleep(1000);

	/* 	Communication Initialization: Once the system processors have been loaded, the application prepares for communication by initializing the system processors.	*/
	LOG_DEBUG("TransputerInitializeSystem...");
	if (TransputerInitializeSystem(OPTO_LOG_ERRORS_FLAG | OPTO_LOG_MESSAGES_FLAG) != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	}

	/* make sure system is optotrak certus
	*/
	LOG_DEBUG("DetermineSystem...");
	if (uDetermineSystem() != OPTOTRAK_CERTUS_FLAG)
	{
		return get_error();
	}

	return NO_ERROR_STR;
}

string OptotrakCertus::init_devices(int* nMarkers)
{
	//memcpy(nMarkersToActivate, nMarkers, 3 * sizeof(int));

	/* Strober Initialization: retreive configuration		*/
	LOG_DEBUG("DetermineStroberConfiguration...");

	if (DetermineStroberConfiguration(&pdtDeviceHandles, &pdtDeviceHandlesInfo, &nDevices) != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	}


	/* 	* check if any devices have been detected by the system 	*/
	if (nDevices == 0)
	{
		return string("Error: No devices detected.\n");
	}
	//if (nDevices == 4) // this is probably a mistake as we don't have more than three strobers connected.
	//{
	//	nDevices = 3; // fixing the error manualy.
	//}

	nMarkersToActivate = new int[nDevices];
	memcpy(nMarkersToActivate, nMarkers, nDevices * sizeof(int));

	ApplicationStoreDeviceProperties(&pdtDevices, pdtDeviceHandlesInfo, nDevices);

	/* 	* Change the number of markers to fire for all devices 	*/
	LOG_DEBUG("set number of markers for each strober...");
	for (int nCurDevice = 0; nCurDevice < nDevices; nCurDevice++)
	{
		activate_device(nCurDevice, nMarkersToActivate[nCurDevice]);
	}	

	return NO_ERROR_STR;
}

string OptotrakCertus::activate_device(int device, int num_markers)
{
	LOG_DEBUG(string("Markers for strober") + to_string(device) + string(" : ") + to_string(num_markers));
	if (pdtDevices[device].bHasROM)
		LOG_DEBUG(string("Device ") + to_string(device) + string(" has ROM"));
	else
		SetMarkersToActivateForDevice(&(pdtDevices			[device]), 
										pdtDeviceHandlesInfo[device].pdtHandle->nID, num_markers);
	return NO_ERROR_STR;
}

string OptotrakCertus::set_optional_flags()
{	
	LOG_DEBUG("setting optional flags (OptotrakSetProcessingFlags) ...");
	if (OptotrakSetProcessingFlags(OPTO_LIB_POLL_REAL_DATA |
		OPTO_CONVERT_ON_HOST |
		OPTO_RIGID_ON_HOST))
		return get_error();
	return NO_ERROR_STR;
}

string OptotrakCertus::load_camera_parameters()
{
	char* camera_params = "standard";
	LOG_DEBUG(string("loading camera parameters (OptotrakLoadCameraParameters): ") + string(camera_params));
	if (OptotrakLoadCameraParameters(camera_params) != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	} /* if */

	return NO_ERROR_STR;
}

string OptotrakCertus::init_data_collection()
{
	LOG_DEBUG("checking number of total markers...");
	if (dtSettings.nMarkers == 0)
	{
		LOG_DEBUG("Error: There are no markers to be activated");
		return get_error();
	}

	LOG_DEBUG("allocate memory for marker data...");
	p3dData = (Position3d*)malloc(dtSettings.nMarkers * sizeof(Position3d));

	LOG_DEBUG("data collection paramters (dtSettings):");	
	char settingsBuffer[200];
	sprintf(settingsBuffer, "%d, %.2f, %.0f, %d, %d, %d, %.2f, %.2f, %.0f, %.0f\n",
		dtSettings.nMarkers,
		dtSettings.fFrameFrequency,
		dtSettings.fMarkerFrequency,
		dtSettings.nThreshold,
		dtSettings.nMinimumGain,
		dtSettings.nStreamData,
		dtSettings.fDutyCycle,
		dtSettings.fVoltage,
		dtSettings.fCollectionTime,
		dtSettings.fPreTriggerTime);
	LOG_DEBUG(settingsBuffer);

	LOG_DEBUG("setting up collection parameters (OptotrakSetupCollection)...");
	if (OptotrakSetupCollection(dtSettings.nMarkers,
		dtSettings.fFrameFrequency,
		dtSettings.fMarkerFrequency,
		dtSettings.nThreshold,
		dtSettings.nMinimumGain,
		dtSettings.nStreamData,
		dtSettings.fDutyCycle,
		dtSettings.fVoltage,
		dtSettings.fCollectionTime,
		dtSettings.fPreTriggerTime,
		OPTOTRAK_NO_FIRE_MARKERS_FLAG | OPTOTRAK_BUFFER_RAW_FLAG | OPTOTRAK_SWITCH_AND_CONFIG_FLAG) != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	}

	sleep(100);

	/*  	* Deactivate the Optotrak markers... will be reactivated later 	*/
	LOG_DEBUG("Optotrak DeActivate Markers...");
	if (OptotrakDeActivateMarkers() != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	}
	return NO_ERROR_STR;
}

string OptotrakCertus::init_rigid_bodies(int* rigidBodyStartMarkers)
{	
	string err;
	const char* stylus = Config::config["stylus"].c_str();
	const char* upper = Config::config["rigid_upper"].c_str(); //"../rigid_bodies/try4_itero/maxilla3_local_to_FORMandible";//"../rigid_bodies/try3_local/maxilla3_local.rig";
	const char* lower = Config::config["rigid_lower"].c_str(); //"../rigid_bodies/try4_itero/mandible3_local.rig";

	if ((err = load_rigid_body_from_file(stylus, rigidBodyStartMarkers[Config::stylus_index])) != NO_ERROR_STR)
		return err;

	if ((err = load_rigid_body_from_file(upper, rigidBodyStartMarkers[Config::upper_index])) != NO_ERROR_STR)
		return err;
	
	if ((err = load_rigid_body_from_file(lower, rigidBodyStartMarkers[Config::lower_index])) != NO_ERROR_STR)
		return err;
	
	
	return NO_ERROR_STR;
}

string OptotrakCertus::read_landmark_files()
{
	const char* file_sphereCenters_lower = (Config::config["artisynth_lower"]).c_str();
	const char* file_lower_digitiezed = (Config::config["digitize_lower"]).c_str();
	const char* file_upper_digitized = (Config::config["digitize_upper"]).c_str();
	const char* file_sphereCenters_upper = (Config::config["artisynth_upper"]).c_str();

	try 
	{
		sphere_centers_lower = read_tabDelimited_points(file_sphereCenters_lower);
		lower_digitized = read_tabDelimited_points(file_lower_digitiezed);		

		sphere_centers_upper = read_tabDelimited_points(file_sphereCenters_upper);
		upper_digitized = read_tabDelimited_points(file_upper_digitized);		
	}
	catch (exception e)
	{
		return LOG_DEBUG("xyz file not found");
	}

	return NO_ERROR_STR;
}

string OptotrakCertus::load_rigid_body_from_file(const char *file, int firstMarker)
{	
	LOG_DEBUG("Adding rigid body from file: " + string(file));
	if (RigidBodyAddFromFile(
		this->NumberOfRigidBodies,   /* ID associated with this rigid body.*/
		firstMarker,              /* First marker in the rigid body.*/
		(char*)file,        /* RIG file containing rigid body coordinates.*/
		OPTOTRAK_QUATERN_RIGID_FLAG |
		OPTOTRAK_RETURN_QUATERN_FLAG))
	{
		return string("init_rigid_bodies(): rigid body initialization failed.\n");
	} /* if */
	LOG_DEBUG("rigid body " + string(file) + " loaded to ID: " + to_string(NumberOfRigidBodies));

	this->NumberOfRigidBodies++;

	return NO_ERROR_STR;
}

string OptotrakCertus::set_FOR(int rigidBodyID)
{
	sleep(1000);
	LOG_DEBUG("Changing FOR...");
	if (rigidBodyID != 0)
	{
		if (RigidBodyChangeFOR(rigidBodyID, OPTOTRAK_CONSTANT_RIGID_FLAG))
		{
			return LOG_DEBUG(get_error());
		}
		LOG_DEBUG("Frame of reference changed to rigidBodyID: " + std::to_string(rigidBodyID));
	}
	return NO_ERROR_STR;
}

void setCursorPosition(int x, int y)
{
	static const HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	std::cout.flush();
	COORD coord = { (SHORT)x, (SHORT)y };
	SetConsoleCursorPosition(hOut, coord);
}

string OptotrakCertus::collect_data()
{				
	cout << "Press any key to start collecting data. \n";
	getchar();
	
	rigidBodyData = new OptotrakRigidStruct[this->NumberOfRigidBodies];
	unsigned int						uFlags,
										uElements,
										uFrameNumber;

	//RigidBodyDataType*	RigidBodyData_rb = new RigidBodyDataType[2];

	/* 	* Activate markers. 	*/
	LOG_DEBUG(("Optotrak Activate Markers..."));
	if (OptotrakActivateMarkers() != OPTO_NO_ERROR_CODE)
	{
		return get_error();
	}

	LOG_DEBUG("---------------------------------Collecting data---------------------------------");	
	
	sleep(1500);
	char input;
	clock_t recording_start_time = clock();
	double time_diff = 0;
	double  first_recorded_time = 0;
	double last_recorded_time = 0;
	bool stop_recording = false;
	bool write_tracking_flag = false;
	bool not_started_writing = true;
	string err;
	system("cls");
	Transform<double, 3, Affine>* certusTransformT = new Transform<double, 3, Affine>[NumberOfRigidBodies];
	Transform<double, 3, Affine>* final_reg = new Transform<double, 3, Affine>[NumberOfRigidBodies];
	for (int nLoop = 0; !stop_recording; nLoop++)
	{			
		setCursorPosition(0, 0);

		if (DataGetLatest3D(&uFrameNumber, &uElements, &uFlags, p3dData) != OPTO_NO_ERROR_CODE)
		{
			return get_error();
		}
		display_markers_data(uFrameNumber, uElements, uFlags);
		cout << "\n";

		time_diff = (clock() - recording_start_time) / (double)CLOCKS_PER_SEC;
		/*cout << "time (sec): " << time_diff << endl;*/
		if (DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, rigidBodyData, 0) != OPTO_NO_ERROR_CODE)
		{
			return get_error();
		}		
		display_rigidBodies(uFrameNumber, uElements, uFlags);

		for (int i = Config::lower_index; i >= Config::upper_index; --i) //ignore the stylus on ID = 0
		{
			QuatTransformation q = rigidBodyData[i].transformation.quaternion;			
			certusTransformT[i] = optotrakTransform_2_EigenTransform(q);
		}

		// write raw transformations
		if (write_tracking_flag && Config::write_raw_flag)
		{
			//write_tracking(final_reg, certusTransformT, i, time);
			if ((err = write_tracking(certusTransformT, time_diff, "raw")) != NO_ERROR_STR)
			{
				LOG_DEBUG(err);
				return err;
			}
			last_recorded_time = time_diff;
		}
		else
		{
			cout << "                                                       \n";
			cout << "                                                       \n";
		}

		// calculate final transformations
		if (Config::register_on_the_fly)
		{
			for (int i = Config::lower_index; i >= Config::upper_index; --i) //ignore the stylus on ID = 0
			{								
				if (i == Config::upper_index)
				{
					final_reg[i] = virtual_2_physical(upper_digitized, sphere_centers_upper, certusTransformT[i]);
				}
				else if (i == Config::lower_index)
				{
					final_reg[i] = virtual_2_physical(lower_digitized, sphere_centers_lower, certusTransformT[i]);
				}
			}

			// write final transformations
			if (write_tracking_flag && Config::write_tranformations_flag)
			{
				//write_tracking(final_reg, certusTransformT, i, time);
				if ((err = write_tracking(final_reg, time_diff, "final")) != NO_ERROR_STR)
				{
					LOG_DEBUG(err);
					return err;
				}
				last_recorded_time = time_diff;
			}
			else
			{
				cout << "                                                       \n";
				cout << "                                                       \n";
			}

			if (Config::register_on_the_fly)
				send_transformations(final_reg, uFrameNumber);
		}
		
		cout << "----------------------------------------------------------------\n";
		Sleep(Config::sleep_time_capture);		
		
		vrpn_server->mainloop();		

		// Keyboard input handler
		if (kbhit())
		{
			input = getch();
			switch (input)
			{
			case 'q':
			case 'Q':
				stop_recording = true;
				break;
			case 'p':
			case 'P':
				cin.get();
				break;
			case 'r':
			case 'R':
				write_tracking_flag = true;								
				if ((err = init_write_tracking()) != NO_ERROR_STR)
					return err;
				recording_start_time = clock();					
				break;
			case 't':
			case 'T':
				write_tracking_flag = false;
				close_finalize_tracking_files();				
				break;
			}
		}
	}	

	// close program
	close();

	return string();
}

string OptotrakCertus::send_transformations(Transform<double, 3, Affine>* final_reg, uint uFrameNumber)
{
	string err;
	cout << "sending Quaternions\n";	
	//for (int i = this->NumberOfRigidBodies-1; i >= 1; --i) //ignore the stylus on ID = 0
	for (int i = Config::lower_index; i >= Config::upper_index; --i) //ignore the stylus on ID = 0
	{		
		//QuatTransformation q = rigidBodyData[i].transformation.quaternion;			
		if ((err = vrpn_server->pack_transformation(final_reg[i], uFrameNumber, i)) != NO_ERROR_STR)
		{
			LOG_DEBUG(err);
			return err;
		}
	}
	return NO_ERROR_STR;
}

string OptotrakCertus::write_tracking(Transform<double, 3, Affine>* transforms, double time, string type)
{
	cout << "Writing tracking into files \n";
	cout << "time (sec): " << time << endl;
	
	try
	{
		if (Config::write_format == "ArtiSynth_probe")
		{
			for (int rigidID = Config::lower_index; rigidID >= Config::upper_index; --rigidID) //ignore the stylus on ID = 0			
				if (type == "final")
				{
					if (rigidID == Config::upper_index)
						write_transformation(ofs_track_trans_upper, time, transforms[rigidID]);
					else if (rigidID == Config::lower_index)
						write_transformation(ofs_track_trans_lower, time, transforms[rigidID]);
				}
				else if (type == "raw")
				{
					if (Config::write_raw_flag)
					{
						if (rigidID == Config::upper_index)
							write_transformation(ofs_track_raw_upper, time, transforms[rigidID]);
						else if (rigidID == Config::lower_index)
							write_transformation(ofs_track_raw_lower, time, transforms[rigidID]);
					}
				}
			
		}
		else
			return string("Error: write format ") + Config::write_format + string(" not defined.");
	}
	catch (exception e)
	{
		LOG_DEBUG(e.what());
		return "Unhandled exception in write_tracking: " + string(e.what());
	}

	return NO_ERROR_STR;
}

Transform<double, 3, Affine> OptotrakCertus::virtual_2_physical(MatrixXd digitized, 
																MatrixXd sphere_centers, 
																Transform<double, 3, Affine>& certusTransformT)
{
	// transform the digitized points to its current location	
	Matrix<double, 3, Dynamic> matrix = digitized.transpose();
	Matrix<double, Dynamic, 3> digitized_transformed = (certusTransformT * matrix).transpose();
	
	// register the spheres onto the new digitized locations 
	Transform<double, 3, Affine> final_registration = procrustes_register(sphere_centers, digitized_transformed);
	
	if (Config::debug_output_flag) cout << "digitized transpose  \n" << digitized.transpose().matrix() << endl;
	if (Config::debug_output_flag) cout << "certusTransformT  \n" << certusTransformT.matrix() << endl;
	if (Config::debug_output_flag) cout << "lower digitized transformed  \n" << digitized_transformed.matrix() << endl;
	if (Config::debug_output_flag) cout << "lower final reg  \n" << final_registration.matrix() << endl;
	
	return final_registration;
}

string OptotrakCertus::GetSdkVersion()
{
	const int BUFSIZE = 200;
	char version[BUFSIZE + 1];
	version[BUFSIZE] = 0;
	OAPIGetVersionString(version, BUFSIZE);
	return string(version);
}

void OptotrakCertus::display_rigidBodies(uint uFrameNumber, uint uElements, uint uFlags)
{
	fprintf(stdout, "Rigid Bodies: %d)\n", this->NumberOfRigidBodies);
	/*fprintf(stdout, "Frame Number: %8u\n", uFrameNumber);
	fprintf(stdout, "Elements    : %8u\n", uElements);
	fprintf(stdout, "Flags       : 0x%04x\n\n", uFlags);*/
	for (int rb = 0; rb < this->NumberOfRigidBodies; ++rb)
	{
		display_rigid_body(rigidBodyData[rb]);
	}
	
}

void OptotrakCertus::display_markers_data(uint uFrameNumber, uint uElements, uint uFlags)
{
	fprintf(stdout, "3D Data Display (%d Markers)\n", dtSettings.nMarkers);	
	fprintf(stdout, "Frame Number: %8u\n", uFrameNumber);
	//fprintf(stdout, "Elements    : %8u\n", uElements);
	//fprintf(stdout, "Flags       : 0x%04x\n", uFlags);
	for (nCurMarker = 0; nCurMarker < dtSettings.nMarkers; nCurMarker++)
	{
		DisplayMarker(nCurMarker + 1, this->p3dData[nCurMarker]);
		
	} /* for */
}

void OptotrakCertus::sleep(unsigned int uSec)
{
	Sleep(uSec);
}

string OptotrakCertus::get_error()
{
	string str;
	if (OptotrakGetErrorString(szNDErrorString, MAX_ERROR_STRING_LENGTH + 1) == 0)
	{				
		str = string(szNDErrorString);		
	}
	else
	{
		str = string("Error is unknown");		
	}
	LOG_DEBUG(str);
	return str;	
}

void OptotrakCertus::close_finalize_tracking_files()
{
	if (ofs_track_raw_lower.is_open())
	{
		ofs_track_raw_lower.close();
		finalize_tracking_file(raw_file_lower);
	}
	if (ofs_track_trans_upper.is_open())
	{
		ofs_track_trans_upper.close();
		finalize_tracking_file(trans_file_upper);
	}
	if (ofs_track_raw_upper.is_open())
	{
		ofs_track_raw_upper.close();
		finalize_tracking_file(raw_file_upper);
	}
	if (ofs_track_trans_lower.is_open())
	{
		ofs_track_trans_lower.close();
		finalize_tracking_file(trans_file_lower);
	}
}

string OptotrakCertus::finalize_tracking_file(string file)
{
	try
	{
		ifstream ifs(file);
		std::string str((std::istreambuf_iterator<char>(ifs)),
			std::istreambuf_iterator<char>());
		ifs.close();
		//int num_lines = lines_count(str);
		ofstream ofs(file);
		ofs << "0.0 " << last_recorded_time << ' ' << "1.0" << endl;
		ofs << str;
		ofs.close();
	}
	catch (exception e)
	{
		return string("Error in finalize_tracking_file: ") + string(e.what());
	}

	return NO_ERROR_STR;
}

string OptotrakCertus::close()
{
	close_finalize_tracking_files();	

	//if (this->Recording)
	StopRecording();
	fprintf(stdout, "...TransputerShutdownSystem\n");
	OptotrakDeActivateMarkers();
	TransputerShutdownSystem();

	fprintf(stdout, "free memory\n");
	if (pdtDeviceHandlesInfo)
	{
		for (int nCurDevice = 0; nCurDevice < nDevices; nCurDevice++)
		{
			AllocateMemoryDeviceHandleProperties(&(pdtDeviceHandlesInfo[nCurDevice].grProperties), 0);
		} /* for */
	} /* if */
	AllocateMemoryDeviceHandles(&pdtDeviceHandles, 0);
	AllocateMemoryDeviceHandlesInfo(&pdtDeviceHandlesInfo, pdtDeviceHandles, 0);

	return NO_ERROR_STR;	
}

string OptotrakCertus::VRPN_server_initiate(const char* tracker_name)
{
	m_Connection = new vrpn_Connection_IP();
	vrpn_server = new VRPN_Server(tracker_name, m_Connection);

	return NO_ERROR_STR;
}

//----------------------------------------------------------------------------
// Enable all tool ports that have tools plugged into them.
string OptotrakCertus::EnableToolPorts()
{
	
	int toolCounter = 0;

	// reset our information about the tool ports
	for (toolCounter = 0; toolCounter < VTK_CERTUS_NTOOLS; toolCounter++)
	{
		if (this->PortEnabled[toolCounter])
		{
			LOG_DEBUG(string("disabling tool ") + to_string(toolCounter));
			if (RigidBodyDelete(this->PortHandle[toolCounter]) != OPTO_NO_ERROR_CODE)
			{
				return get_error();
			}
		}
		this->PortEnabled[toolCounter] = 0;
	}

	// stop tracking
	//if (this->Recording)
	//{
	//	LOG_DEBUG("DeActivating Markers");
	//	if (!OptotrakDeActivateMarkers() != OPTO_NO_ERROR_CODE)
	//	{
	//		return get_error();
	//	}
	//	this->Recording = false;
	//}

	// device handles (including status)
	int nDeviceHandles = 0;
	DeviceHandle *deviceHandles = nullptr;

	int allDevicesEnabled = 0;
	for (int trialNumber = 0;
		trialNumber < 3 && !allDevicesEnabled;
		trialNumber++)
	{
		LOG_DEBUG("Getting Number Device Handles");
		if (OptotrakGetNumberDeviceHandles(&nDeviceHandles) != OPTO_NO_ERROR_CODE)
		{
			return get_error();
		}

		if (nDeviceHandles <= 0)
		{
			LOG_DEBUG("EnableToolPorts: no Optotrack strobers found");
			return get_error();
		}

		// get all device handles and the status of each one
		deviceHandles = new DeviceHandle[nDeviceHandles];

		unsigned int flags = 0;
		LOG_DEBUG(string("Getting Device Handles for ") + to_string(nDeviceHandles) + string(" devices"));
		if (OptotrakGetDeviceHandles(deviceHandles, nDeviceHandles, &flags) != OPTO_NO_ERROR_CODE)
		{			
			delete[] deviceHandles;
			return get_error();
		}

		// initialize this to 1 (set to 0 if unenabled handles found)
		allDevicesEnabled = 1;

		// free any unoccupied handles, enable any initialized handles
		for (int deviceCounter = 0;
			deviceCounter < nDeviceHandles;
			deviceCounter++)
		{
			int ph = deviceHandles[deviceCounter].nID;
			DeviceHandleStatus status = deviceHandles[deviceCounter].dtStatus;

			if (status == DH_STATUS_UNOCCUPIED)
			{
				LOG_DEBUG(string("Delete port handle ") + to_string(ph));
				if (OptotrakDeviceHandleFree(ph) != OPTO_NO_ERROR_CODE)
				{
					return get_error();
				}
				allDevicesEnabled = 0;
			}
			else if (status == DH_STATUS_INITIALIZED)
			{
				LOG_DEBUG(string("Enable port handle ") + to_string(ph));
				if (OptotrakDeviceHandleEnable(ph) != OPTO_NO_ERROR_CODE)
				{
					return get_error();
				}
				// enabling a strober will make other tools appear,
				// so let's be paranoid and always set this to zero
				allDevicesEnabled = 0;
			}
		}
	}

	// reset the number of markers and rigid bodies to zero
	//this->NumberOfMarkers = 0;
	

	// get information for all tools
	for (int deviceCounter = 0;
		deviceCounter < nDeviceHandles;
		deviceCounter++)
	{
		int ph = deviceHandles[deviceCounter].nID;
		DeviceHandleStatus status = deviceHandles[deviceCounter].dtStatus;

		if (status == DH_STATUS_UNOCCUPIED)
		{
			// this shouldn't happen, but just in case
			continue;
		}

		DeviceHandleProperty *properties = 0;
		int nProperties = 0;
		LOG_DEBUG(string("Getting number of properties for port handle ") + to_string( ph));
		if (OptotrakDeviceHandleGetNumberProperties(ph, &nProperties)
			!= OPTO_NO_ERROR_CODE
			|| nProperties == 0)
		{
			return get_error();
		}
		else
		{
			properties = new DeviceHandleProperty[nProperties];
			LOG_DEBUG(string("Getting ")+ to_string( nProperties ) + string( " properties for handle " ) + to_string (ph));
			if (OptotrakDeviceHandleGetProperties(ph, properties, nProperties)
				!= OPTO_NO_ERROR_CODE)
			{
				return get_error();
			}
			else
			{
				// the properties of interest
				static const int deviceNameMaxlen = 128;
				char deviceName[deviceNameMaxlen + 1];
				int hasROM = 0;
				int nToolPorts = 0;
				int nSwitches = 0;
				int nVLEDs = 0;
				int nSubPort = 0;
				int nMarkersToFire = 0;
				int status = 0;

				for (int propCounter = 0; propCounter < nProperties; propCounter++)
				{
					unsigned int propertyID = properties[propCounter].uPropertyID;
					if (propertyID == DH_PROPERTY_NAME)
					{
						strncpy(deviceName, properties[propCounter].dtData.szData,
							deviceNameMaxlen);
						deviceName[deviceNameMaxlen] = '\0';
					}
					else if (propertyID == DH_PROPERTY_HAS_ROM)
					{
						hasROM = properties[propCounter].dtData.nData;
					}
					else if (propertyID == DH_PROPERTY_TOOLPORTS)
					{
						nToolPorts = properties[propCounter].dtData.nData;
					}
					else if (propertyID == DH_PROPERTY_SWITCHES)
					{
						nSwitches = properties[propCounter].dtData.nData;
					}
					else if (propertyID == DH_PROPERTY_VLEDS)
					{
						nVLEDs = properties[propCounter].dtData.nData;
					}
					else if (propertyID == DH_PROPERTY_SUBPORT)
					{
						nSubPort = properties[propCounter].dtData.nData;
					}
					else if (propertyID == DH_PROPERTY_MARKERSTOFIRE)
					{
						nMarkersToFire = properties[propCounter].dtData.nData;
					}
					else if (propertyID == DH_PROPERTY_STATUS)
					{
						status = properties[propCounter].dtData.nData;
					}
				}

				// verify that this is a tool, and not a strober
				if (hasROM && nToolPorts == 0)
				{
					// assume only one strober: index tools by SubPort
					int port = nSubPort - 1;

					LOG_DEBUG(string("Found tool port ")+to_string(port)+ string(" for device ") + string(deviceName));

					if (port >= 0 && port < VTK_CERTUS_NTOOLS)
					{
						if (this->PortEnabled[port] &&
							this->PortHandle[port] != ph)
						{
							LOG_DEBUG(string("Port number ")+to_string(port )+ 
								string(" is already taken by a different tool"));
						}
						else
						{
							this->PortHandle[port] = ph;
							this->PortEnabled[port] = (status == DH_STATUS_ENABLED);

							LOG_DEBUG(string("Port ") + to_string(port) + string(" enabled for tool ") +
								string(deviceName) + string(" with ") + to_string(nMarkersToFire) 
								+ string(" markers"));

							/*std::ostringstream toolPortName;
							toolPortName << port;*/
							//vtkPlusDataSource* trackerTool = NULL;
							/*if (this->GetToolByPortName(toolPortName.str().c_str(), trackerTool) != PLUS_SUCCESS)
							{
								LOG_DEBUG(string("Warning: Undefined connected tool found in the strober on port '")
									+ string(toolPortName.str()) + string("' with name '") + string( deviceName )
									+string( "', disabled it until not defined in the config file: " ));
								this->PortEnabled[port] = 0;
							}*/
						}

						//this->NumberOfMarkers += nMarkersToFire;
					}

					delete[] properties;
				}
			}
		}
	} //end for (int deviceCounter = 0;...)

	if (deviceHandles)
	{
		delete[] deviceHandles;
	}

	this->NumberOfRigidBodies = 0;
	//this->RigidBodyMap.clear();
	// add rigid bodies
	for (toolCounter = 0; toolCounter < VTK_CERTUS_NTOOLS; toolCounter++)
	{
		if (this->PortEnabled[toolCounter])
		{
			int ph = this->PortHandle[toolCounter];
			int rigidID = this->NumberOfRigidBodies;
			LOG_DEBUG(string("Adding rigid body for port handle")+ to_string(ph));
			if (RigidBodyAddFromDeviceHandle(ph,
				rigidID, // rigID is port handle
				OPTOTRAK_QUATERN_RIGID_FLAG |
				OPTOTRAK_RETURN_QUATERN_FLAG)
				!= OPTO_NO_ERROR_CODE)
			{
				return get_error();
			}
			else
			{
				//this->RigidBodyMap[rigidID] = toolCounter;
				// increment the number of rigid bodies
				this->NumberOfRigidBodies++;
/*
				std::ostringstream toolPortName;
				toolPortName << toolCounter;
				vtkPlusDataSource* trackerTool = NULL;
				if (this->GetToolByPortName(toolPortName.str().c_str(), trackerTool) != PLUS_SUCCESS)
				{
					LOG_ERROR("Failed to get tool by port name: " << toolPortName.str());
					continue;
				}*/
			}
		}
	}

	// re-start the tracking
	//if (this->Recording)
	//{
		LOG_DEBUG("Activating Markers");
		if (OptotrakActivateMarkers() != OPTO_NO_ERROR_CODE)
		{
			return get_error();
		} 		
	//}

	

	return NO_ERROR_STR;
}

string OptotrakCertus::StopRecording()
{
	if (OptotrakDeActivateMarkers() != OPTO_NO_ERROR_CODE)
	{
		return this->get_error();
	}

	if (this->DisableToolPorts() != NO_ERROR_STR)
	{
		return this->get_error();
	}

	//this->Recording = false;
	return NO_ERROR_STR;
}

string OptotrakCertus::DisableToolPorts()
{
	/*if (this->Recording)
	{*/
		if (OptotrakDeActivateMarkers() != OPTO_NO_ERROR_CODE)
		{
			return get_error();
		}
	//}

	// disable the enabled ports
	for (int toolCounter = 0; toolCounter < VTK_CERTUS_NTOOLS; toolCounter++)
	{
		if (this->PortEnabled[toolCounter])
		{
			if (RigidBodyDelete(this->PortHandle[toolCounter]) != OPTO_NO_ERROR_CODE)
			{
				return get_error();
			}
		}
		this->PortEnabled[toolCounter] = 0;
	}

	//// re-start the tracking
	//if (this->Recording)
	//{
	//	if (this->init_markers_tools() != NO_ERROR_STR)
	//	{
	//		return get_error();
	//	}
	//}
	return NO_ERROR_STR;
}

string	OptotrakCertus::digitize_rigid_body(int rigidID)
{
	char yesno;
	cout << "Do you want to digitize rigid body: #" << rigidID << "? (Y/N)" << endl;
	cin >> yesno;
	char input;
	if (yesno == 'y')
	{
		LOG_DEBUG("Digitizing points for rigid body: " + to_string(rigidID));
		set_FOR(rigidID);

		int num;
		cout << "Number of points to digitize? ";
		cin >> num;
		Position3d* points = new Position3d[num];
		int pointCounter = 0;
		
		unsigned int uFlags, uElements, uFrameNumber;
		LOG_DEBUG(("Optotrak Activate Markers..."));
		if (OptotrakActivateMarkers() != OPTO_NO_ERROR_CODE) 	{	return get_error();	}
		cout << "During data collecting, press the 'C' key to record the point.\n Press enter to start collection.\n";
		for (int nLoop = 0; ; nLoop++)
		{
			
			system("cls");
			//if (!this->Recording)
				//LOG_DEBUG("collect_data() stopped because Recording == false");

			if (DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, rigidBodyData, 0) != OPTO_NO_ERROR_CODE)
			{
				return get_error();
			}
			display_rigidBodies(uFrameNumber, uElements, uFlags);
			cout << "----------------------------------------------------------------\n";
			
			if (kbhit())
			{
				input = getch();
				if (input == 'c' || input == 'C')
				{
					points[pointCounter++] = rigidBodyData[0].transformation.quaternion.translation;
					if (pointCounter >= num)
						break;
				}
			}
			cout << "points recorded: " << pointCounter << endl;
		}

		cout << pointCounter << " points recorded for rigidID " << rigidID << endl;
		ofstream file;		
		file.open(digitizedFileNames[rigidID].c_str());
		for (int i = 0; i < num; ++i)
		{
			file << points[i].x << "\t" << points[i].y << "\t" << points[i].z << endl;
		}
		file.close();
		cout << "digitiezed points recorded in file: " << digitizedFileNames[rigidID] << endl;
		delete points;
	}
	else
	{
		return NO_ERROR_STR;
	}

	return NO_ERROR_STR;
}

string	OptotrakCertus::digitize()
{
	if (stylusID == -1)
	{
		cout << "Which port(ID) is Stylus (default = 0)?  ";
		cin >> stylusID;
	}
	for (int i = 0; i < NumberOfRigidBodies; ++i)
	{
		if (i != stylusID)
			digitize_rigid_body(i);
	}
	
	return NO_ERROR_STR;
}