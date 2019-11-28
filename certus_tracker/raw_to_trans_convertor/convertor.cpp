#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <map>
#include <io.h>
#include "utils.h"
#include <windows.h>
#include <stdio.h>
#include <assert.h>

using namespace Eigen;
using namespace std;

int main() //read and register tracking files
{

	// -----------------CONFIG	-------------------------------------------
	string data_folder = "../../data/study2_subjectBenedikt/fiducials/";
	string tracking_path = "../../data/study2_subjectBenedikt/trajectories/test_conversion/";

	string file_sphereCenters_lower = data_folder + "lower_spheres_artisynth.xyz";
	string file_sphereCenters_upper = data_folder + "upper_spheres_artisynth.xyz";
	string file_lower_digitiezed = data_folder + "digitize_lower_certus.xyz";
	string file_upper_digitized = data_folder + "digitize_upper_certus.xyz";

	// -----------------READ LANDMARK FILES-------------------------------------------
	MatrixXd							lower_digitized;
	MatrixXd							upper_digitized;
	MatrixXd							sphere_centers_lower;
	MatrixXd							sphere_centers_upper;

	sphere_centers_lower = read_tabDelimited_points(file_sphereCenters_lower.c_str());
	lower_digitized = read_tabDelimited_points(file_lower_digitiezed.c_str());
	sphere_centers_upper = read_tabDelimited_points(file_sphereCenters_upper.c_str());
	upper_digitized = read_tabDelimited_points(file_upper_digitized.c_str());

	// -----------------READ TRACKING FILE-------------------------------------------	
	// just get the raw files
	_finddata_t file_data;
	int ff = _findfirst((tracking_path + "*raw*.trck").c_str(), &file_data);
	if (ff != -1)
	{
		int res = 0;
		while (res != -1)
		{
			cout << "File #:" << res + 1 << endl;
			//cout << data.name << endl;
			string tracking_file = file_data.name;
			string file = tracking_path + file_data.name;
			//file += tracking_file;?
			int num_samples = 0;
			double* times;
			auto tracking = read_tracking_file(file.c_str(), times, num_samples, 2);
			cout << tracking_file << endl;
			// -----------------REGISTER and write-------------------------------------------
			// create output filename
			string output_file_str = tracking_file;
			int index = output_file_str.find("raw", 0);
			if (index == std::string::npos) break;
			output_file_str.replace(index, 3, "trans");
			string ofile = tracking_path + output_file_str.c_str();

			// create trans file
			ofstream		ofs_track_trans;
			ofs_track_trans.open(ofile);
			ofs_track_trans << "0.0 " << times[num_samples - 1] << " " << "1.0" << endl;
			ofs_track_trans << "linear 7 explicit\n";

			for (size_t i = 0; i < num_samples; i++)
			{
				Transform<double, 3, Affine> final_reg;
				string cast = tracking_file.substr(0, 5);
				if (tracking_file.find("lower") != std::string::npos)
					final_reg = virtual_2_physical(lower_digitized, sphere_centers_lower, tracking[i]);
				else if (tracking_file.find("upper") != std::string::npos)
					final_reg = virtual_2_physical(upper_digitized, sphere_centers_upper, tracking[i]);
				else
				{
					cout << "tracking file was neither upper nor lower!" << endl;
					assert(false);
				}

				write_transformation(ofs_track_trans, times[i], final_reg);
			}

			ofs_track_trans.close();
			cout << "File #" << res + 1 << " converted to " << output_file_str << endl;

			res = _findnext(ff, &file_data);
		}
		_findclose(ff);		
	}
	else
		cout << "No file found" << endl;


	return 0;
}