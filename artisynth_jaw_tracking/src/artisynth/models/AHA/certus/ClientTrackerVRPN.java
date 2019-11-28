package artisynth.models.AHA.certus;
import java.io.*;

import com.sun.org.apache.xpath.internal.operations.Bool;

import vrpn.*;
import artisynth.core.mechmodels.RigidBody;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.graph.Vertex;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.Quaternion;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

public class ClientTrackerVRPN implements vrpn.TrackerRemote.PositionChangeListener	
{	
	public void trackerPositionUpdate( TrackerRemote.TrackerUpdate u, TrackerRemote tracker )
	{
		System.out.println( "Tracker position message from vrpn: \n" +
				"\ttime:  " + u.msg_time.getTime( ) + "  sensor:  " + u.sensor + "\n" +
				"\tposition:  " + u.pos[0] + " " + u.pos[1] + " " + u.pos[2] + "\n" +
				"\torientation:  " + u.quat[0] + " " + u.quat[1] + " " +
				u.quat[2] + " " + u.quat[3] );


		int t = u.sensor-1;
		Quaternion q = new Quaternion(	u.quat[0], u.quat[1], u.quat[2], u.quat[3]);
		Vector3d v = new Vector3d(u.pos[0], u.pos[1], u.pos[2]);

		RotationMatrix3d rotMat = new RotationMatrix3d(q);
		rotMat.transpose();
		RigidTransform3d new_trans = new RigidTransform3d(v, rotMat);

		certusDemo.update_cast_position(t,  new_trans);
	}

	CertusTracking certusDemo;
	public ClientTrackerVRPN(CertusTracking _certusDemo)
	{
		certusDemo = _certusDemo;				
	}	
}