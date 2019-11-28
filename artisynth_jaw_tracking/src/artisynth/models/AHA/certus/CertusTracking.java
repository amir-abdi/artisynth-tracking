package artisynth.models.AHA.certus;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.PlyReader;
import maspack.matrix.AxisAngle;
import maspack.matrix.Line;
import maspack.matrix.Point3d;
import maspack.matrix.Quaternion;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;
import maspack.render.color.JetColorMap;
import vrpn.TrackerRemote;
import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.CollisionManager.ColliderType;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ScalarRange;
import artisynth.core.workspace.RootModel;

public class CertusTracking extends RootModel  {
	ClientTrackerVRPN client_vrpn;	
	MechModel mechModel;
	RigidBody lower_body;
	RigidBody upper_body;
	RigidBody casts_rigidBody[];
	Point3d trackPoint = new Point3d(Config.PointToTrack);
	Point3d prevTrackPoint = new Point3d(Config.PointToTrack);	
	long currentTime = 0;
	int trackSphereCounter = 0;
	private static Color CREAM = createColor (255, 255, 200);
	private static Color GOLD = createColor (255, 150, 0);

	public CertusTracking()
	{
		super();
		setName ("Certus Demo");

		firstTime[0] = true;
		firstTime[1] = true;
	}   


	public void InitVRPN_Client(String server)
	{
		String trackerName = server;
		TrackerRemote tracker = null;
		BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
		try
		{
			tracker = new TrackerRemote( trackerName, null, null, null, null );
		}
		catch( InstantiationException e )
		{ 
			System.out.println( "We couldn't connect to tracker " + trackerName + "." );
			System.out.println( e.getMessage( ) );
			try
			{
				System.out.flush();
				System.err.flush();
				System.out.println( "hit enter to end" );
				in.readLine();
			}
			catch( IOException ioe ) {}
			return;
		}

		ClientTrackerVRPN test = new ClientTrackerVRPN(this);
		tracker.addPositionChangeListener( test );                         
	}

	@Override
	public void build (String[] args) throws IOException {
		super.build (args);      

		mechModel = new MechModel ("MasticationMechModel");
		this.addModel (mechModel);
		lower_body =  utils.add_cast (mechModel, "lower_body", 
				Config.lowerFileName, Config.ENAMEL_DENSITY);		
		upper_body =  utils.add_cast (mechModel, "upper_body", 
				Config.upperFileName, Config.ENAMEL_DENSITY);		
		casts_rigidBody = new RigidBody[2];
		casts_rigidBody[0] = upper_body;
		casts_rigidBody[1] = lower_body;

		mechModel.setGravity(new Vector3d(0,0,0));
		if (Config.CollisionFlag)
			collision_init();

		if (Config.DataSource == "vrpn")
			InitVRPN_Client(Config.ServerName);
		else if (Config.DataSource == "probe")
			InitInputProbe();
		AddControlPanels();		
	}		

	private void InitInputProbe() {
		try{
			String[] props = new String[2];
			props[0] = new String("position");
			props[1] = new String("rotation");
			 
			NumericInputProbe probe_lower = new NumericInputProbe (
					            lower_body, props, Config.data_path + Config.probe_path_lower);
//					            0.0, 38.375);
			
			probe_lower.setName("probe_lower");
			addInputProbe(probe_lower);
			
			NumericInputProbe probe_upper = new NumericInputProbe (
		            upper_body, props, Config.data_path + Config.probe_path_upper);
//		            0.0, 38.375);

			probe_upper.setName("probe_upper");
			addInputProbe(probe_upper);				
			System.out.println(probe_lower.getVsize());
		}
		catch(Exception ex)
		{
			System.out.println("cannot create probe: " +  ex.getMessage());
		}
		
		
		
	}

//	public StepAdjustment advance(double t0, double t1, int flags) {
//		//		System.out.print("BaseMod:\t");
//		//		System.out.println(System.nanoTime()-currentTime);
//		currentTime = System.nanoTime();
//
//		if (Config.VisualizeSinglePoint)
//		{
//			trackSphereCounter++;
//			if (trackSphereCounter % Config.TrackPointFrameRate == 0)
//			{
//				RigidBody sph = utils.add_sphere(mechModel, String.format ("sph%1$1s_track", trackSphereCounter), 0.2, 100, 100, trackPoint, Color.cyan);
//				sph.setDynamic(false);
//			}
//			prevTrackPoint.set(trackPoint);
//		}
//
//		StepAdjustment ret = super.advance(t0, t1, flags);
//		return ret;
//	}

	boolean firstTime[] = new boolean[2];	
	RigidTransform3d prev_trans[] = new RigidTransform3d[2];

	public void update_cast_position(int cast_index, RigidTransform3d transform)
	{
		//move the lower cast a bit upper for more collision l as a test case
		//		if (index==1)
		//		{
		//			transform.p.x += 0.2;
		//		}

//		transform.R.transpose();
		
		if (Config.RemoveJitter)
		{
			if (firstTime[cast_index])
			{
				firstTime[cast_index] = false;
				prev_trans[cast_index] = new RigidTransform3d(transform);
				casts_rigidBody[cast_index].setPose(transform);
			}
			else
			{
				RigidTransform3d temp = new RigidTransform3d(prev_trans[cast_index]);
				temp.invert();
				temp.mul(transform);
				double s = 0;
				s = temp.R.m01 +   temp.R.m02 +    temp.R.m10 +   temp.R.m12 + temp.R.m20 +   temp.R.m21 ;
				s += temp.p.x +temp.p.y +temp.p.z;  
				//				System.out.println(s);

				if (Math.abs(s)>0.25)
				{
					prev_trans[cast_index] = new RigidTransform3d(transform);
					casts_rigidBody[cast_index].setPose(transform);
				}
			}
		}
		else
			casts_rigidBody[cast_index].setPose(transform);		
		

//		if (cast_index==0)			
//		{
//			transform.R.transpose();
//			trackPoint.set(Config.PointToTrack);			
//			trackPoint.transform(transform);
//			//			System.out.println(trackPoint);
//		}
		
		if (cast_index == 0)
		{
			//			System.out.print("Tracker:\t");		
			//			System.out.println(System.nanoTime()-currentTime);		
			currentTime = System.nanoTime();
		}
	}

	public void collision_init () {
		// create and set a collision behavior between body0 and body1, and make
		// collisions INACTIVE since we only care about graphical display
		CollisionBehavior behav = new CollisionBehavior (true, 0);

		behav.setMethod (CollisionBehavior.Method.INACTIVE);
		behav.setDrawPenetrationDepth (0); // show penetration of mesh 0
		behav.getPenetrationDepthRange().setUpdating (
				ScalarRange.Updating.AUTO_FIT); 										// todo: play with this one******
		mechModel.setCollisionBehavior (lower_body, upper_body, behav);

		CollisionManager cm = mechModel.getCollisionManager();
		// penetration rendering only works with contour-based collisions
		cm.setColliderType (ColliderType.AJL_CONTOUR);							 
		// set other rendering properities in the collision manager:
		RenderProps.setVisible (cm, true);    // enable collision rendering
		cm.setDrawIntersectionContours(true); // draw contours ...
		RenderProps.setEdgeWidth (cm, 3);     // with a line width of 3
		RenderProps.setEdgeColor (cm, Color.RED); // and a blue color
		// create a custom color map for rendering the penetration depth
		JetColorMap map = new JetColorMap();
		map.setColorArray (
				new Color[] {
						CREAM,                       // no penetration
						createColor (255, 204, 153),
						createColor (255, 153, 102),
						createColor (255, 102, 51),
						createColor (255, 51, 0),
						createColor (204, 0, 0),     // most penetration
				});
		cm.setColorMap (map);

		// create a separate color bar to show depth values associated with the
		// color map
		ColorBar cbar = createColorBar();
		cbar.setColorMap (map);
	}

	// color bar for collision depth
	@Override
	public void prerender(RenderList list) {
		// In prerender, we update the color bar labels based on the updated
		// penetration range stored in the collision behavior.
		//
		// Object references are obtained by name using 'findComponent'. This is
		// more robust than using class member variables, since the latter will
		// be lost if we save and restore this model from a file.
		if (Config.CollisionFlag)
		{
			ColorBar cbar = (ColorBar)(renderables().get("colorBar"));            
			CollisionBehavior behav = mechModel.getCollisionBehavior(lower_body, upper_body);
			ScalarRange range = behav.getPenetrationDepthRange();
			cbar.updateLabels(0, 1000*range.getUpperBound());
			// call the regular prerender method
		}
		
		
		if (Config.DataSource == "probe")
		{
			Quaternion q = lower_body.getRotation();
			Vector3d v = lower_body.getPosition();
			RotationMatrix3d rotMat = new RotationMatrix3d(q);
			rotMat.transpose();
			RigidTransform3d new_trans = new RigidTransform3d(v, rotMat);	
			lower_body.setPose(new_trans);

			System.out.println("Lower:");
			System.out.println(q);
			System.out.println(v);

			q = upper_body.getRotation();
			v = upper_body.getPosition();
			rotMat = new RotationMatrix3d(q);
			rotMat.transpose();
			new_trans = new RigidTransform3d(v, rotMat);	
			upper_body.setPose(new_trans);

			System.out.println("Upper:");
			System.out.println(q);
			System.out.println(v);

			//		System.out.println("source changed");
		}
		
		super.prerender(list); 
	}

	//Control Panels
	// Convenience method for creating colors from [0-255] RGB values
	private static Color createColor (int r, int g, int b) {
		return new Color (r/255.0f, g/255.0f, b/255.0f);
	}         

	// Creates and returns a ColorBar renderable object
	public ColorBar createColorBar() {
		ColorBar cbar = new ColorBar();
		cbar.setName("colorBar");
		cbar.setNumberFormat("%.2f");      // 2 decimal places
		cbar.populateLabels(0.0, 1.0, 10); // Start with range [0,1], 10 ticks
		cbar.setLocation(-100, 0.1, 20, 0.8);
		cbar.setTextColor (Color.WHITE);
		addRenderable(cbar);               // add to root model's renderables
		return cbar;
	}      

	public void AddControlPanels () 
	{
		AddVisibilityContorlPanel();		
	}

	public void AddVisibilityContorlPanel()
	{
		ControlPanel VisibilityPanel = new ControlPanel ("options");
		VisibilityPanel.addWidget ("upper_visible", this, "castVisibilityUpper");
		VisibilityPanel.addWidget ("lower_visible", this, "castVisibilityLower");
		addControlPanel (VisibilityPanel);		
		Main.getMain ().arrangeControlPanels (this);
	}

	protected boolean castVisibilityLower = true;
	public boolean getCastVisibilityLower () {
		return castVisibilityLower;
	}

	public void setCastVisibilityLower (boolean visible) {	      
		RenderProps.setVisible (lower_body, visible);	      
		castVisibilityLower = visible;
	}

	protected boolean castVisibilityUpper = true;
	public boolean getCastVisibilityUpper () {
		return castVisibilityUpper;
	}

	public void setCastVisibilityUpper (boolean visible) {	      
		RenderProps.setVisible (upper_body, visible);	      
		castVisibilityUpper = visible;
	}

	public static PropertyList myProps = new PropertyList (CertusTracking.class, RootModel.class);
	public PropertyList getAllPropertyInfo() {
		return myProps;
	}
	static {
		myProps.add ("castVisibilityLower", "visibility of lower cast", true);
		myProps.add ("castVisibilityUpper", "visibility of upper cast", true);
	}
}
