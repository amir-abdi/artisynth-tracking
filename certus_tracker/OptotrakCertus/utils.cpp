#include "utils.h"
//Inclide ND Library Files and Application files 
//#include "certus_aux.c"
//#include "ot_aux.c"

string log_file = "";
//string tracking_trans_file = "";
//string tracking_raw_file = "";


void check_directory(const char* dir_name)
{
	struct stat info;
	if (stat(dir_name, &info) != 0)
	{
		_mkdir(dir_name);
		cout << dir_name << " directory created\n";
	}
}

string LOG_DEBUG(string str)
{
	std::ofstream file;
	file.open(get_log_file(), std::ios_base::app);
	file << get_current_time_date() << '\t' << str << endl;
	cout << get_current_time_date() << '\t' << str << endl;
	file.flush();
	file.close();
	return str;
}

string LOG_DEBUG(char* str)
{
	return LOG_DEBUG(string(str));
}

string LOG_DEBUG(stringstream ss)
{
	return LOG_DEBUG(ss.str());
}

string get_tracking_file(string pre_jaw, string mid_type)
{
	string dir_name = Config::config["tracking_directory"];
	check_directory(dir_name.c_str());
	
	string date_time = get_current_time_date();
	string tracking_trans_file = dir_name + pre_jaw + "_" + mid_type + "_" + date_time + ".trck";
	
	return tracking_trans_file;
}

string get_log_file()
{
	auto dir_name = Config::config["log_directory"];
	if (log_file == "")
	{
		string date_time = get_current_time_date();
		log_file = dir_name + "log_" + date_time + ".log";
	}
	else
		return log_file;

	struct stat info;
	
	check_directory(dir_name.c_str());
	
	return log_file;
}

string get_current_time_date()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y-%m-%d__%H;%M;%S", timeinfo);
	std::string str(buffer);

	return str;
}

std::array<double, 16> ndiTransformToMatrixd(const double trans[8])
{
	double ww, xx, yy, zz, wx, wy, wz, xy, xz, yz, ss, rr, f;
	std::array<double, 16> matrix;

	/* Determine some calculations done more than once. */
	ww = trans[0] * trans[0];
	xx = trans[1] * trans[1];
	yy = trans[2] * trans[2];
	zz = trans[3] * trans[3];
	wx = trans[0] * trans[1];
	wy = trans[0] * trans[2];
	wz = trans[0] * trans[3];
	xy = trans[1] * trans[2];
	xz = trans[1] * trans[3];
	yz = trans[2] * trans[3];

	rr = xx + yy + zz;
	ss = (ww - rr)*0.5;
	/* Normalization factor */
	f = 2.0 / (ww + rr);

	/* Fill in the matrix. */
	matrix[0] = (ss + xx)*f;
	matrix[1] = (wz + xy)*f;
	matrix[2] = (-wy + xz)*f;
	matrix[3] = 0;
	matrix[4] = (-wz + xy)*f;
	matrix[5] = (ss + yy)*f;
	matrix[6] = (wx + yz)*f;
	matrix[7] = 0;
	matrix[8] = (wy + xz)*f;
	matrix[9] = (-wx + yz)*f;
	matrix[10] = (ss + zz)*f;
	matrix[11] = 0;
	matrix[12] = trans[4];
	matrix[13] = trans[5];
	matrix[14] = trans[6];
	matrix[15] = 1;

	return matrix;
}

std::array<double, 16> Quaternion_2_TransformationMatrix(QuatTransformation q, double q_error)
{
	std::array<double, 16> matrix;
	double transform[8];
	double *trans = transform;
	trans[0] = q.rotation.q0;
	trans[1] = q.rotation.qx;
	trans[2] = q.rotation.qy;
	trans[3] = q.rotation.qz;
	trans[4] = q.translation.x;
	trans[5] = q.translation.y;
	trans[6] = q.translation.z;
	trans[7] = q_error;	

	double ww, xx, yy, zz, wx, wy, wz, xy, xz, yz, ss, rr, f;
	/* Determine some calculations done more than once. */
	ww = trans[0] * trans[0];
	xx = trans[1] * trans[1];
	yy = trans[2] * trans[2];
	zz = trans[3] * trans[3];
	wx = trans[0] * trans[1];
	wy = trans[0] * trans[2];
	wz = trans[0] * trans[3];
	xy = trans[1] * trans[2];
	xz = trans[1] * trans[3];
	yz = trans[2] * trans[3];

	rr = xx + yy + zz;
	ss = (ww - rr)*0.5;
	/* Normalization factor */
	f = 2.0 / (ww + rr);

	/* Fill in the matrix. */
	matrix[0] = (ss + xx)*f;
	matrix[1] = (wz + xy)*f;
	matrix[2] = (-wy + xz)*f;
	matrix[3] = 0;
	matrix[4] = (-wz + xy)*f;
	matrix[5] = (ss + yy)*f;
	matrix[6] = (wx + yz)*f;
	matrix[7] = 0;
	matrix[8] = (wy + xz)*f;
	matrix[9] = (-wx + yz)*f;
	matrix[10] = (ss + zz)*f;
	matrix[11] = 0;
	matrix[12] = trans[4];
	matrix[13] = trans[5];
	matrix[14] = trans[6];
	matrix[15] = 1;

	return matrix;	
}

MatrixXd read_MeshLab_point_select(char* file, int count)
{
	std::ifstream t(file);
	std::string str((std::istreambuf_iterator<char>(t)),
		std::istreambuf_iterator<char>());
	MatrixXd matrix(count, 3);
	xml_document<char> doc;
	doc.parse<0>((char*)str.c_str());		
	int i = 0;
	xml_node<> *picked_points = doc.first_node();
	for (xml_node<> *node = picked_points->first_node("point");
		node; node = node->next_sibling("point"), i++)
	{
		double x = atof((node->first_attribute("x"))->value());
		double y = atof((node->first_attribute("y"))->value());
		double z = atof((node->first_attribute("z"))->value());		
		matrix(i, 0) = x;
		matrix(i, 1) = y;
		matrix(i, 2) = z;
	}	

	if (Config::debug_output_flag) cout << "\nselected point set from mesh lab:" << endl << matrix << endl;
	return matrix;
}

static long lines_count(string s)
{
	long count = 1;
	int position = 0;
	while ((position = s.find('\n', position)) != -1)
	{
		count++;
		position++;         // Skip this occurrence!
		if (position >= s.length())
		{
			count--;
			break;
		}
	}
	return count;
}

MatrixXd read_tabDelimited_points(const char* file)
{
	std::ifstream t(file);
	std::string str((std::istreambuf_iterator<char>(t)),
		std::istreambuf_iterator<char>());	
	int count = lines_count(str);	
	MatrixXd matrix(count, 3);
	stringstream ss(str);
	for (int i = 0; i < count; ++i)
	{
		ss >> matrix(i, 0);
		ss >> matrix(i, 1);
		ss >> matrix(i, 2);
	}
	if (Config::debug_output_flag) cout << "\n tab delimited matrix:\n" << matrix << endl;
	return matrix;
}

Transform<double, 3, Affine>* read_tracking_file(const char* file, 
												double*& times,
												int& num_samples, 
												int header_skip)
{
	std::ifstream t(file);
	std::string str((std::istreambuf_iterator<char>(t)),
		std::istreambuf_iterator<char>());
	int count = lines_count(str);
	//MatrixXd matrix(count, 7);
	Transform<double, 3, Affine>* transforms =
		new Transform<double, 3, Affine>[count - header_skip];
	num_samples = count - header_skip;
	times = new double[num_samples];

	stringstream ss(str);	
	for (int i = 0; i < count; ++i)
	{
		if (i < header_skip)
		{
			string line;
			getline(ss, line);
			if (Config::debug_output_flag) cout << "skipped header\n";
			continue;
		}
		ss >> times[i-header_skip];
		if (Config::debug_output_flag) cout << "read time: " << times[i - header_skip] << endl;

		Matrix<double, 3, 1> qV; // = Vector3d(q.translation.x, q.translation.y, q.translation.z);
		ss >> qV[0];
		ss >> qV[1];
		ss >> qV[2];

		Quaternion<double> qQ; // = Quaternion<double>(q.rotation.q0, q.rotation.qx, q.rotation.qy, q.rotation.qz);
		ss >> qQ.w();
		ss >> qQ.x();
		ss >> qQ.y();
		ss >> qQ.z();

		transforms[i - header_skip].fromPositionOrientationScale(qV, qQ, Vector3d(1, 1, 1));
	}	
	return transforms;
}

Transform<double, 3, Affine> optotrakTransform_2_EigenTransform(QuatTransformation q)
{
	Transform<double, 3, Affine> trans;
	Matrix<double, 3, 1> qV = Vector3d(q.translation.x, q.translation.y, q.translation.z);
	Quaternion<double> qQ = Quaternion<double>(q.rotation.q0, q.rotation.qx, q.rotation.qy, q.rotation.qz);
	trans.fromPositionOrientationScale(qV, qQ, Vector3d(1, 1, 1));

	return trans;
}

QuatTransformation EigenTransform_2_optotrakTransform(Transform<double, 3, Affine>& q)
{
	QuatTransformation outputQ;	
	outputQ.translation.x = q.translation().x();
	outputQ.translation.y = q.translation().y();
	outputQ.translation.z = q.translation().z();
	Quaternion<double> quatOut;
	quatOut = q.rotation();	
	outputQ.rotation.q0 = quatOut.w();
	outputQ.rotation.qx = quatOut.x();
	outputQ.rotation.qy = quatOut.y();
	outputQ.rotation.qz = quatOut.z();

	return outputQ;
}

Transform<double, 3, Affine> procrustes_register(MatrixXd movingMat, MatrixXd refMat)
{	
	double scale;
	Matrix3d R;
	Vector3d t;
	
	igl::procrustes(movingMat, refMat, false, false, scale, R, t);
	assert(scale == 1);
	
	// check results
	MatrixXd movingMatPrime = (movingMat * R).rowwise() + t.transpose();

	if (Config::debug_output_flag) cout << "\nMatrix movingMat:\n" << movingMat << endl;
	if (Config::debug_output_flag) cout << "\nMatrix refMat :\n" << refMat<< endl;
	if (Config::debug_output_flag) cout << "\nMatrix movingMat transformed into:\n" << movingMatPrime << endl;

	double diff = 0;	
	for (int i = 0; i < movingMatPrime.rows(); i++)
	{
		//double t = (movingMatPrime.row(i) - refMat.row(i)).norm();
		diff += (movingMatPrime.row(i)-refMat.row(i)).squaredNorm();			
	}
	diff /= movingMatPrime.rows();
	diff = sqrt(diff);
	if (Config::debug_output_flag) 
		cout << "RMS procrustes registration: " << diff << endl;
	//assert(diff < 3);
	
	if (Config::debug_output_flag) cout << "matrix X:\n" << movingMat << endl;
	if (Config::debug_output_flag) cout << "matrix Y:\n" << refMat << endl;
	if (Config::debug_output_flag) cout << "X*R:\n" << (movingMat * R) << endl;
	
	if (Config::debug_output_flag) cout << "\nrotation matrix:\n" << R << endl;
	if (Config::debug_output_flag) cout << "\ntranslation vector: \n" << t << endl;
	Transform<double, 3, Affine> trans;
	trans.fromPositionOrientationScale(t, R, Vector3d(1,1,1));
	/*trans = R;
	trans.translate(t);*/
	if (Config::debug_output_flag) cout << "transformation matrix: \n" << trans.matrix() << endl;
	
	return trans;
}

//Display Functions
void DisplayMatrix3x3(RotationMatrixType	matrix)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			DisplayFloat(matrix[i][j]);
		cout << endl;
	}
}

void Display4d(QuatRotation dtPosition3d)
{
	fprintf(stdout, "q0 ");
	DisplayFloat(dtPosition3d.q0);
	fprintf(stdout, "qx ");
	DisplayFloat(dtPosition3d.qx);
	fprintf(stdout, "qy ");
	DisplayFloat(dtPosition3d.qy);
	fprintf(stdout, "qz ");
	DisplayFloat(dtPosition3d.qz);

}

void DisplayArray16(array<double, 16> element)
{
	for (int i = 0; i < 16; ++i)
	{
		cout << element[i] << ' ';
		if (i % 4 == 3)
			cout << endl;
	}
	cout << endl;
}

void display_rigid_body(OptotrakRigidStruct rigidBody)
{
	fprintf(stdout, "Rigid Body: %d\n", rigidBody.RigidId);
	/*cout << "Rotation Matrix: \n";
	DisplayMatrix3x3(rigidBody.transformation.rotation.matrix);
	cout << "\nRotation Translation: ";
	DisplayPosition3d(rigidBody.transformation.rotation.translation);*/
	cout << "quaternion rotation: ";
	Display4d(rigidBody.transformation.quaternion.rotation);
	cout << "\nquaternion Translation: ";
	DisplayPosition3d(rigidBody.transformation.quaternion.translation);
	//cout << endl;

	array<double, 16> element = Quaternion_2_TransformationMatrix(rigidBody.transformation.quaternion,
		rigidBody.QuaternionError);

	/*cout << "transformed matrix: \n";
	DisplayArray16(element);
	*/cout << endl;

}

void display_quaternion(QuatTransformation q)
{
	cout << "quaternion rotation: ";
	Display4d(q.rotation);
	cout << "\nquaternion Translation: ";
	DisplayPosition3d(q.translation);
	cout << endl;
}

void DisplayFloat(float fFloat)
{
	if (fFloat < MAX_NEGATIVE)
	{
		fprintf(stdout, "%10s%5s", "MISSING", "");
	}
	else
	{
		fprintf(stdout, "%10.2f%5s", fFloat, "");
	} /* if */
} /* DisplayFloat */


void DisplayPosition3d(Position3d dtPosition3d)
{
	fprintf(stdout, "X ");
	DisplayFloat(dtPosition3d.x);
	fprintf(stdout, "Y ");
	DisplayFloat(dtPosition3d.y);
	fprintf(stdout, "Z ");
	DisplayFloat(dtPosition3d.z);

} /* DisplayPosition3d */


void DisplayMarker(int nMarker, Position3d dtPosition3d)
{
	fprintf(stdout, "Marker_%.3d: ", nMarker);
	DisplayPosition3d(dtPosition3d);
	fprintf(stdout, "\n");

} /* DisplayMarker */


string write_transformation(ofstream& ofs, double time, Transform<double, 3, Affine>& q)
{
	ofs << time << ' ';
	ofs << q.translation().x() << ' ';
	ofs << q.translation().y() << ' ';
	ofs << q.translation().z() << ' ';	

	Quaternion<double> quatOut;
	quatOut = q.rotation();
	ofs << quatOut.w() << ' ';
	ofs << quatOut.x() << ' ';
	ofs << quatOut.y() << ' ';
	ofs << quatOut.z() << endl;

	return NO_ERROR_STR;
}