/*
My first vrpn client - vrpnHelloWorld
If you want to transform a CAMERA, VIEWPOINT or HMD, instead of an displayed object,
you need to invert the transform, since
vrpn returns the transform sensor to tracker/base/emitter transform.

// NOTE: a vrpn tracker must call user callbacks with tracker data (pos and
//       ori info) which represent the transformation xfSourceFromSensor.
//       This means that the pos info is the position of the origin of
//       the sensor coord sys in the source coord sys space, and the
//       quat represents the orientation of the sensor relative to the
//       source space (ie, its value rotates the source's axes so that
//       they coincide with the sensor's)
*/

#include <stdlib.h>
#include <stdio.h>
#include <vrpn_tracker.h>

/*****************************************************************************
*
Callback handler
*
*****************************************************************************/

void    VRPN_CALLBACK handle_tracker(void *userdata, const vrpn_TRACKERCB t)
{
	

	printf("handle_tracker\tSensor %d is now at position(%g,%g,%g)\n",
		t.sensor,
		t.pos[0], t.pos[1], t.pos[2]);
	printf("handle_tracker\tSensor %d is now at rotation(%g,%g,%g,%g)\n",
		t.sensor,
		t.quat[0], t.quat[1], t.quat[2], t.quat[3]);

}

//****************************************************************************
//
//   Main function
//
//****************************************************************************

int main(int argc, char *argv[])

{
	int     done = 0;
	vrpn_Tracker_Remote *tkr;

	// Open the tracker

	tkr = new vrpn_Tracker_Remote("Tracker0@localhost");
	// Set up the tracker callback handler

	tkr->register_change_handler(NULL, handle_tracker);

	// the handle_tracker fucntion will be called whenever the
	// tracker position for ANY of the tracker's sensors are updated.
	// if you are interested in only specific sensors (this should be
	// the most common case), use this method instead:

	// tkr->register_change_handler(NULL, handle_tracker,2);
	// handle_tracker will be called only when sensor #2 is updated.


	//
	// main interactive loop
	//

	while (!done) {
		// Let the tracker do it's thing
		// It will call the callback funtions you registered above
		// as needed
		system("cls");
		tkr->mainloop();
		Sleep(100);
	}
}   //main