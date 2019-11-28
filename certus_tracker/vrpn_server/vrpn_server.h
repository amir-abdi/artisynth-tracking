#pragma once
#include "vrpn_Text.h"
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "quat/quat.h"
#include <math.h>
#include <cstdlib>

#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <stdio.h>                      // for fprintf, stderr, printf, etc

#include "vrpn_Configure.h"             // for VRPN_CALLBACK, etc
#include "vrpn_Shared.h"                // for vrpn_SleepMsecs
#include "vrpn_Types.h"                 // for vrpn_float64
using namespace std;

class vrpn_server_opto : public vrpn_Tracker
{
public:
	vrpn_server_opto(const char* str, vrpn_Connection *c);
	virtual ~vrpn_server_opto() {};
	void set_send_data(int);
	virtual void mainloop();

protected:
	struct timeval _timestamp;
	//vrpn_Connection* vrpn_connection;
};

