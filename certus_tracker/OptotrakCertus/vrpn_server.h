#pragma once
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <stdio.h>                      // for fprintf, stderr, printf, etc
#include <array>
#include <string>

#include "vrpn_Configure.h"             // for VRPN_CALLBACK, etc
#include "vrpn_Shared.h"                // for vrpn_SleepMsecs
#include "vrpn_Types.h"                 // for vrpn_float64
#include "vrpn_Text.h"
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "quat/quat.h"
#include "ndtypes.h"

#include "constants.h"
#include "utils.h"
using namespace std;



class VRPN_Server : public vrpn_Tracker
{
public:
	VRPN_Server(const char* str, vrpn_Connection *c);	
	virtual ~VRPN_Server() {};
	string pack_transformation(QuatTransformation, uint, int);
	string pack_transformation(Transform<double, 3, Affine>& q, uint frame_number, int rigidID = 0);
	virtual void mainloop();
	string send_transformation(array<double, 16>);

protected:
	timeval _timestamp;
	vrpn_Connection* vrpn_connection;
};

