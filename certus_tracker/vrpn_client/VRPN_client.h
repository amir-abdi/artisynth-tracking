#pragma once
#include "vrpn_Tracker.h"

class VRPN_Client
{
public:
	VRPN_Client(char* str);
	~VRPN_Client() {};
	void VRPN_CALLBACK VRPN_Client::handle_tracker(void* userData, const vrpn_TRACKERCB t);
	virtual void mainloop();
	vrpn_Tracker_Remote* tr;

protected:
	struct timeval _timestamp;
};

