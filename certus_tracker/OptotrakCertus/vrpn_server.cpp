#include "vrpn_server.h"

VRPN_Server::VRPN_Server(const char* str, vrpn_Connection *c) : vrpn_Tracker(str, c)
{
	vrpn_connection = c;
}

string VRPN_Server::pack_transformation(QuatTransformation q, uint frame_number, int rigidID)
{
	vrpn_gettimeofday(&_timestamp, NULL);
	vrpn_Tracker::timestamp = _timestamp;
	vrpn_Tracker::frame_count = frame_number;
	
	//// We will just put a fake data in the position of our tracker
	//static float angle = 0; angle += 0.001f;

	// the pos array contains the position value of the tracker
	// XXX Set your values here
	
	pos[0] = q.translation.x;
	pos[1] = q.translation.y;
	pos[2] = q.translation.z;

	// the d_quat array contains the orientation value of the tracker, stored as a quaternion
	//// XXX Set your values here
	d_quat[0] = q.rotation.q0;
	d_quat[1] = q.rotation.qx;
	d_quat[2] = q.rotation.qy;
	d_quat[3] = q.rotation.qz;
	

	char msgbuf[1000];

	d_sensor = rigidID;


	int  len = vrpn_Tracker::encode_to(msgbuf);

	if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf,
		vrpn_CONNECTION_LOW_LATENCY))
	{
		string err = "can't write message: tossing\n";
		return err;
	}
	return NO_ERROR_STR;
}

string VRPN_Server::pack_transformation(Transform<double, 3, Affine>& q, uint frame_number, int rigidID)
{
	vrpn_gettimeofday(&_timestamp, NULL);
	vrpn_Tracker::timestamp = _timestamp;
	vrpn_Tracker::frame_count = frame_number;

	//// We will just put a fake data in the position of our tracker
	//static float angle = 0; angle += 0.001f;

	// the pos array contains the position value of the tracker
	// XXX Set your values here

	pos[0] = q.translation().x();
	pos[1] = q.translation().y();
	pos[2] = q.translation().z();

	// the d_quat array contains the orientation value of the tracker, stored as a quaternion
	////// XXX Set your values here
	//d_quat[0] = q.rotation()[0];
	//d_quat[1] = q.rotation()[1];
	//d_quat[2] = q.rotation()[2];
	//d_quat[3] = q.rotation()[3];

	Quaternion<double> quatOut;
	quatOut = q.rotation();
	d_quat[0] = quatOut.w();
	d_quat[1] = quatOut.x();
	d_quat[2] = quatOut.y();
	d_quat[3] = quatOut.z();
	

	char msgbuf[1000];

	d_sensor = rigidID;	

	int  len = vrpn_Tracker::encode_to(msgbuf);

	if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf,
		vrpn_CONNECTION_LOW_LATENCY))
	{
		string err = "can't write message: tossing\n";
		return err;
	}
	return NO_ERROR_STR;
}

void VRPN_Server::mainloop()
{	
	server_mainloop();
	vrpn_connection->mainloop();
}

string VRPN_Server::send_transformation(array<double, 16>)
{	
	return string("sending 4x4 transformation matrix is not implemented\n");
}