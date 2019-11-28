#include <fstream>
#include "igl/procrustes.h"
#include <iostream>
using namespace Eigen;
using namespace std;

#pragma once
static long lines_count(string s);
int write_transformation(ofstream& ofs, double time, Transform<double, 3, Affine>& q);
Transform<double, 3, Affine> procrustes_register(MatrixXd movingMat, MatrixXd refMat);

MatrixXd read_tabDelimited_points(const char* file);
Transform<double, 3, Affine>* read_tracking_file(const char* file,
	double*& times,
	int& num_samples,
	int header_skip);
Transform<double, 3, Affine> virtual_2_physical(MatrixXd digitized,
	MatrixXd sphere_centers,
	Transform<double, 3, Affine>& certusTransformT);