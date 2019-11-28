#include "vrpn_server.h"

void vrpn_server_opto::set_send_data(int test)
{
	vrpn_gettimeofday(&_timestamp, NULL);

	vrpn_Tracker::timestamp = _timestamp;

	// We will just put a fake data in the position of our tracker
	static float angle = 0; angle += 0.001f;

	// the pos array contains the position value of the tracker
	// XXX Set your values here
	pos[0] = test;
	/*pos[0] = sinf(angle);
	pos[1] = 0.0f;
	pos[2] = 0.0f;*/

	// the d_quat array contains the orientation value of the tracker, stored as a quaternion
	//// XXX Set your values here
	//d_quat[0] = rand();
	//d_quat[1] = 0.0f;
	//d_quat[2] = 0.0f;
	//d_quat[3] = 1.0f;

	char msgbuf[1000];

	d_sensor = 0;


	int  len = vrpn_Tracker::encode_to(msgbuf);

	if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf,
		vrpn_CONNECTION_LOW_LATENCY))
	{
		fprintf(stderr, "can't write message: tossing\n");
	}
}

void vrpn_server_opto::mainloop()
{	
	server_mainloop();
	//vrpn_connection->mainloop();
}

vrpn_server_opto::vrpn_server_opto(const char* str, vrpn_Connection *c) : vrpn_Tracker(str, c)
{
	//vrpn_connection = c;
}

