#pragma once
#include "vrpn_Tracker.h"

class vrpn_common : public vrpn_Tracker
{
public:
	vrpn_common(vrpn_Connection *c = 0);
	virtual ~vrpn_common () {};

	virtual void mainloop();

protected:
	struct timeval _timestamp;
};