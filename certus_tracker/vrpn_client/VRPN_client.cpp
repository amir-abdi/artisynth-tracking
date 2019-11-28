//#include "VRPN_client.h"
//
//void    VRPN_CALLBACK handle_tracker(void *userdata, const vrpn_TRACKERCB t)
//{
//	//this function gets called when the tracker's POSITION xform is updated
//
//	//you can change what this callback function is called for by changing
//	//the type of t in the function prototype above.
//	//Options are:
//	//   vrpn_TRACKERCB              position
//	//   vrpn_TRACKERVELCB           velocity
//	//   vrpn_TRACKERACCCB           acceleration
//	//   vrpn_TRACKERTRACKER2ROOMCB  tracker2room transform
//	//                                 (comes from local or remote
//	//                                  vrpn_Tracker.cfg file)
//	//   vrpn_TRACKERUNIT2SENSORCB   unit2sensor transform (see above comment)
//	//   vrpn_TRACKERWORKSPACECB     workspace bounding box (extent of tracker)
//
//	// userdata is whatever you passed into the register_change_handler function.
//	// vrpn sucks it up and spits it back out at you. It's not used by vrpn internally
//
//	printf("handle_tracker\tSensor %d is now at position(%g,%g,%g)\n",
//		t.sensor,
//		t.pos[0], t.pos[1], t.pos[2]);
//	printf("handle_tracker\tSensor %d is now at rotation(%g,%g,%g,%g)\n",
//		t.sensor,
//		t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
//
//}
//
//
//
//VRPN_Client::VRPN_Client(char* str)
//{
//	tr = new vrpn_Tracker_Remote(str);
//	//tr->register_change_handler(NULL, handle_tracker);
//}
//
//
//void VRPN_Client::mainloop()
//{
//
//	tr->mainloop();
//}
//
