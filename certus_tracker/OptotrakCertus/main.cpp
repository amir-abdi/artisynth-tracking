#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//Inclide ND Library Files and Application files 
#include "ndtypes.h"
#include "ndpack.h"
#include "ndopto.h"
#include "certus_aux.h"
#include "ot_aux.h"
#include "OptotrakCertus.h"
#include "utils.h"
#include <map>
#include "config.h"
//#include <windows.h>
#include <io.h>

using namespace std;

void main_test_xml()
{
	char* file = "..\\..\\data\\lowermodelmeshICPfreeze_picked_points.pp";
	read_MeshLab_point_select(file, 10);
}

void main_procrustes()
{
	

	char* file_ndi = "..\\..\\data\\mandible2_10pts_digitized.txt";
	MatrixXd X = read_tabDelimited_points(file_ndi);

	char* file_meshlab = "..\\..\\data\\lowermodelmeshICPfreeze_picked_points.pp";
	MatrixXd Y = read_MeshLab_point_select(file_meshlab, 10);

	procrustes_register(X, Y);
}


void main() //optotrak certus
{
	Config::InitConfig();
	
	OptotrakCertus optotrak;
	string err;
	int nmarkers[4] = {0, 4, 8}; // zero, stylus, maxilla, mandible
	//int nmarkers[3] = {4, 4, 4};
	int rigidBodyStartMarkers[3] = { 1,5,9 };

	

	if ((err = optotrak.init(nmarkers, rigidBodyStartMarkers)) != NO_ERROR_STR)
	{
		cout << err;
		getchar();
		return;
	}
	cout << "OAPI SDK Version: " << optotrak.GetSdkVersion() << endl;
					
	if ((err = optotrak.VRPN_server_initiate(Config::server_name.c_str())) != NO_ERROR_STR)
	{
		cout << err;
		getchar();
		return;
	}

	// todo: get rid of recording flag!
	//optotrak.Recording = true;
	
	if ((err = optotrak.collect_data()) != NO_ERROR_STR)
	{
		cout << err;
		getchar();
		return;
	}
	
	
}