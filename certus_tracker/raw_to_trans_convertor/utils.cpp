#include "utils.h"

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

int write_transformation(ofstream& ofs, double time, Transform<double, 3, Affine>& q)
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

	return 0;
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

	double diff = 0;
	for (int i = 0; i < movingMatPrime.rows(); i++)
	{
		//double t = (movingMatPrime.row(i) - refMat.row(i)).norm();
		diff += (movingMatPrime.row(i) - refMat.row(i)).squaredNorm();
	}
	diff /= movingMatPrime.rows();
	diff = sqrt(diff);

	Transform<double, 3, Affine> trans;
	trans.fromPositionOrientationScale(t, R, Vector3d(1, 1, 1));
	/*trans = R;
	trans.translate(t);*/

	return trans;
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
			continue;
		}
		ss >> times[i - header_skip];

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
	return matrix;
}

Transform<double, 3, Affine> virtual_2_physical(MatrixXd digitized,
	MatrixXd sphere_centers,
	Transform<double, 3, Affine>& certusTransformT)
{
	// transform the digitized points to its current location	
	Matrix<double, 3, Dynamic> matrix = digitized.transpose();
	Matrix<double, Dynamic, 3> digitized_transformed = (certusTransformT * matrix).transpose();

	// register the spheres onto the new digitized locations 
	Transform<double, 3, Affine> final_registration = procrustes_register(sphere_centers, digitized_transformed);
	return final_registration;
}
