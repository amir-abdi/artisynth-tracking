// vrpn_server.cpp : Defines the entry point for the console application.
//

#include "vrpn_server.h"

int _tmain(int argc, _TCHAR* argv[])
{
	// Creating the network server
	vrpn_Connection_IP* m_Connection = new vrpn_Connection_IP();

	// Creating the tracker
	vrpn_server_opto* serverTracker = new vrpn_server_opto("Tracker0", m_Connection);
	
	cout << "Created VRPN server." << endl;

	while (true)
	{
		serverTracker->set_send_data(10);
		serverTracker->mainloop();	
		m_Connection->mainloop();

		// Calling Sleep to let the CPU breathe.
		SleepEx(1, FALSE);
	}
}
