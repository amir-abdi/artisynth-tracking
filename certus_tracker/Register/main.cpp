//#include "constants.h"
//#include <stdio.h>
//#include <stdlib.h>
#include <iostream>
////Inclide ND Library Files and Application files 
//#include "ndtypes.h"
//#include "ndpack.h"
//#include "ndopto.h"
//#include "certus_aux.h"
//#include "ot_aux.h"
//#include "OptotrakCertus.h"
#include "utils.h"
//#include <map>
#include "config.h"

using namespace std;
using namespace Eigen;

int main()
{
	Config::InitConfig();

	// -----------------READ LANDMARK FILES-------------------------------------------
	MatrixXd							lower_digitized;
	MatrixXd							upper_digitized;
	MatrixXd							sphere_centers_lower;
	MatrixXd							sphere_centers_upper;

	const char* file_sphereCenters_lower = (Config::config["artisynth_lower"]).c_str();
	const char* file_lower_digitiezed = (Config::config["digitize_lower"]).c_str();
	const char* file_upper_digitized = (Config::config["digitize_upper"]).c_str();
	const char* file_sphereCenters_upper = (Config::config["artisynth_upper"]).c_str();

	sphere_centers_lower = read_tabDelimited_points(file_sphereCenters_lower);
	lower_digitized = read_tabDelimited_points(file_lower_digitiezed);

	sphere_centers_upper = read_tabDelimited_points(file_sphereCenters_upper);
	upper_digitized = read_tabDelimited_points(file_upper_digitized);
	
	// -----------------READ TRACKING FILE-------------------------------------------
	const char* tracking_file = "lower_raw_2017-05-19__15;33;13.trck";
	char* tracking_path = "..\\..\\data\\HumanSubject\\certus_outputs\\tracking\\";
	auto tracking = read_tracking_file(strcat(tracking_path, tracking_file), 2);


	// -----------------REGISTER-------------------------------------------
	
	//OptotrakCertus::virtual_2_physical()

}