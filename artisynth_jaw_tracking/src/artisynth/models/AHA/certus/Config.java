package artisynth.models.AHA.certus;

import maspack.matrix.Point3d;

class Config {
	// file paths
	final static String data_path = "C:\\gitRepository\\jaw-tracking\\data\\study1_subjectAmir_\\";

	final static String lowerFileName = "models\\lower.stl";
	final static String upperFileName = "models\\upper.stl";

	static String sphereLocationsOutputFile = data_path + "artisynth_outputs\\";
	static String activeCast = "upper"; // option: lower, upper

	final static String probe_path_lower = "trajectories\\lower_trans_2018-05-02__13;02;18.trck";
	final static String probe_path_upper = "trajectories\\upper_trans_2018-05-02__13;02;18.trck";

	// mesh/body properties
	final static double ENAMEL_DENSITY = 2900; // kg/m^3
	final static double SPHERE_DENSITY = 8000; //kg/m^3
	final static double STYLUS_HEAD_Radius = 2.98/2; //mm
	final static double PENET_TOLERANCE_DEFAULT = 0.02d;
	final static double COL_POINT_TOL_DEFAULT = 0.25;
	final static double COLUMBUS_COEFF_DEFAULT = 0.01d;
	static final double PENTOL_DEFAULT = 1e-8;

	//Track and visualize parameters
	final static Point3d PointToTrack = new Point3d(581.719, 335.672, -702.46);
	final static int TrackPointFrameRate = 1;
	final static boolean CollisionFlag = false;
	final static boolean VisualizeSinglePoint = false;
	final static boolean RemoveJitter = false;

	// application parameters
	final static String ServerName = "certus_final@localhost";

	final static boolean FLE_TEST = false;
	final static boolean SPHERE_METHOD = true;

	final static String DataSource = "vrpn"; //options: probe vrpn	
}