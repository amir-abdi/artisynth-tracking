#pragma once
#include "ndtypes.h"
#include <array>
#include "igl/procrustes.h"
#include <string>
#include <sys/stat.h>
#include <time.h>
#include <direct.h>
#include <fstream>
#include <streambuf>
#include <rapidxml.hpp>
#include <map>
#include "ndtypes.h"
#include "ndpack.h"
#include "ndopto.h"
#include "certus_aux.h"
#include "config.h"

using namespace rapidxml;
using namespace Eigen;
using namespace std;

#define OUTPUT_DEBUG
#include "ot_aux.h"
#include "constants.h"

std::array<double, 16> ndiTransformToMatrixd(const double trans[8]);
std::array<double, 16>  Quaternion_2_TransformationMatrix(QuatTransformation q, double q_error);
MatrixXd read_MeshLab_point_select(char* file, int);
string LOG_DEBUG(string);
string LOG_DEBUG(char* str);
string LOG_DEBUG(stringstream ss);
string get_log_file();
string get_current_time_date();
static long lines_count(string s);
MatrixXd read_tabDelimited_points(const char* file);
Transform<double, 3, Affine>* read_tracking_file(const char* file, double*&, int& num_samples, int header_skip=0);
Transform<double, 3, Affine> procrustes_register(MatrixXd X, MatrixXd Y);
Transform<double, 3, Affine> optotrakTransform_2_EigenTransform(QuatTransformation);
QuatTransformation EigenTransform_2_optotrakTransform(Transform<double, 3, Affine>& q);

//Display Functions
void DisplayMatrix3x3(RotationMatrixType	matrix);
void Display4d(QuatRotation dtPosition3d);
void DisplayArray16(array<double, 16> element);
void display_rigid_body(OptotrakRigidStruct rigidBody);
void display_quaternion(QuatTransformation q);
void DisplayFloat(float fFloat);
void DisplayPosition3d(Position3d dtPosition3d);
void DisplayMarker(int nMarker, Position3d dtPosition3d);

//Write tracking
string get_tracking_file(string, string);
string write_transformation(ofstream&, double, Transform<double, 3, Affine>& q);
void check_directory(const char* dir_name);