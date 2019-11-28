package artisynth.models.AHA.certus;

import java.awt.Button;
import java.awt.Color;
import java.awt.Component;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;

import javax.swing.JButton;
import javax.swing.event.MouseInputListener;

import com.sun.corba.se.spi.logging.LogWrapperFactory;

import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.Quaternion;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.MouseRayEvent;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.NumberFormat;
import vrpn.TrackerRemote;
import artisynth.core.driver.Main;
import artisynth.core.driver.ViewerManager;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.Monitor;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.AHA.certus.*;

import java.awt.event.MouseListener;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;

public class SphereFiducialMarker extends RootModel {
	ClientTrackerVRPN client_vrpn;

	MechModel mechModel;
	RigidBody cast_body;
	SphereCreator sphereCreator;
	PrintWriter pw;

	public SphereFiducialMarker () 
	{
		super ();
		
		setName ("Sphere Landmarks");      
		sphereCreator = new SphereCreator (Main.getMain().getViewerManager());      
		mechModel = new MechModel ("MasticationMechModel");
		setupRenderProps ();  

		String current = System.getProperty("user.dir");
		System.out.println("Current working directory in Java : " + current);
	}

	// add prop
	double columbusCoeff = Config.COLUMBUS_COEFF_DEFAULT; 
	protected double penetrationTol = Config.PENTOL_DEFAULT;

	public PropertyList getAllPropertyInfo() {
		return myProps;
	}
	public static PropertyList myProps = new PropertyList (SphereFiducialMarker.class, RootModel.class);
	static {	      
		myProps.add ("columbusCoeff", "Collision mu value", Config.COLUMBUS_COEFF_DEFAULT, "[0,1]");
		myProps.add ("penetrationTol", "penetration tolerance", Config.PENTOL_DEFAULT);
	}

	public double getPenetrationTol () {
		return penetrationTol;
	}

	public void setPenetrationTol (double penetrationTol) {
		this.penetrationTol = penetrationTol;
		mechModel.setPenetrationTol (penetrationTol);
	}

	public double getColumbusCoeff () {
		return columbusCoeff;
	}

	public void setColumbusCoeff (double mu) {
		if (columbusCoeff != mu) {	         
			columbusCoeff = mu;			
		}
	}
	//

	@Override
	public void attach (DriverInterface driver) {
		driver.getViewerManager().addMouseListener(sphereCreator);
	}

	@Override
	public void build (String[] args) throws IOException 
	{
		super.build (args);

		this.addModel (mechModel); 
		mechModel.setGravity (new Vector3d(0, 0, 0));

		File path = new File(Config.sphereLocationsOutputFile);
		if (!(path.exists() && path.isDirectory()))
		{
			path.mkdirs();
		}
		if (Config.activeCast == "lower")
		{
			cast_body =  utils.add_cast (mechModel, "lower_body", 
					Config.lowerFileName, Config.ENAMEL_DENSITY);
			Config.sphereLocationsOutputFile = Config.sphereLocationsOutputFile  + "lower_spheres.txt";
		}
		else if (Config.activeCast == "upper")
		{
			cast_body =  utils.add_cast (mechModel, "upper_body", 
					Config.upperFileName, Config.ENAMEL_DENSITY);
			Config.sphereLocationsOutputFile = Config.sphereLocationsOutputFile  + "upper_spheres.txt";
		}

		RenderProps.setFaceColor (cast_body, Color.WHITE.darker ());
		//		utils.transformMeshtoLocal (cast_body, new Point3d(549.3476442,	333.6764575,	-702.1378197));//(486.2413, 343.4032, -526.485));
		cast_body.setDynamic (false);
		sphereCreator.set (mechModel, cast_body, this);

		//		RigidBody sph_body = utils.add_sphere(mechModel, "sph_body", 10, 100, 100, new Point3d(0,0,0));//lower_body.getCenterOfMass());      
		//		utils.transformtoLocal(sph_body);
		//		sph_body.setDynamic (false);
		//		mouseHandler.set (mechModel, sph_body, this);       

		addContactRenderControl();
		addSaveSpheresControl();
	}

	public void setupRenderProps() {
		RenderProps.setPointColor(mechModel, Color.CYAN.darker());
		RenderProps.setPointStyle(mechModel, PointStyle.SPHERE);
		RenderProps.setPointRadius(mechModel, 0.005);
		RenderProps.setLineColor(mechModel, Color.BLUE.darker());
		RenderProps.setLineStyle(mechModel, LineStyle.CYLINDER);
		RenderProps.setLineRadius(mechModel, 0.004);
		RenderProps.setFaceColor(mechModel, Color.white.darker());
		mechModel.setExcitationColor(Color.RED.darker());
	}

	public void addSaveSpheresControl () {
		ControlPanel panel = new ControlPanel ("Save Spheres' Positions");
		JButton saveButton = new JButton ("Save Spheres into File");
		JButton closebutton = new JButton ("Close File");
		JButton openbutton = new JButton ("Open File");

		openbutton.addActionListener (new ActionListener() {         
			@Override
			public void actionPerformed (ActionEvent e) {
				openFile();

			}        
		});

		//		panel.addWidget (openbutton);
		saveButton.addActionListener (new ActionListener() {         
			@Override
			public void actionPerformed (ActionEvent e) {
				saveSpheres();            
			}        
		});
		panel.addWidget (saveButton);

		closebutton.addActionListener (new ActionListener() {         
			@Override
			public void actionPerformed (ActionEvent e) {
				closeFile();

			}        
		});
		//		panel.addWidget (closebutton);

		addControlPanel (panel);
		Main.getMain ().arrangeControlPanels (this);
	}

	private void closeFile()
	{
		pw.close();
	}

	private void openFile()
	{

		String sphereFile = Config.sphereLocationsOutputFile;
		try{
			pw = new PrintWriter (sphereFile);
		}
		catch (Exception e) {
			System.out.println ("File " + sphereFile + " not found.");
			return;
		}
	}

	private void saveSpheres () {		
		Main.getMain().pause();
		Main.getMain().waitForStop();

		try(FileWriter fw = new FileWriter(Config.sphereLocationsOutputFile, true);
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter out = new PrintWriter(bw))
		{
			int sph_counter = utils.countSpheres(mechModel);				
			for (int i = 0; i < sph_counter; ++i)
			{
				RigidBody sph = mechModel.rigidBodies ().
						get (String.format ("sph%1$1s_body", i));			
				try {
					sph.getPosition ().write (out, new NumberFormat ("%g"));
					System.out.println(String.format("Sphere %1$1s written to file", sph.getName()));
					out.write ("\n");
					out.flush();				
				}
				catch (Exception e) {
					System.out.println ("Could not write to file ");
					System.out.println(e.getMessage());
					return;
				}
			}
			fw.close();
			System.out.println("sphere locations written into file: " + Config.sphereLocationsOutputFile);


			if (Config.FLE_TEST)
			{
				// delete all spheres		
				mechModel.clearCollisionBehaviors();
				int counterTemp = 0;
				while (sph_counter-- != 0)
				{
					RigidBody sphToBeRemoved = mechModel.rigidBodies ().
							get (String.format ("sph%1$1s_body", counterTemp++));
					if (sphToBeRemoved != null)
					{						
						System.out.println(String.format("Sphere %1$1s removed", sphToBeRemoved.getName()));

						mechModel.removeRigidBody(sphToBeRemoved);
					}
					else
					{					
						sph_counter++;
					}					
				}				
				System.out.println("All spheres removed");

				System.out.println("Remove all particles and axial springs");
				mechModel.axialSprings().removeAll();
				for (Particle p : mechModel.particles()) {
					mechModel.detachPoint(p);
				}				

				mechModel.particles().removeAll();
			}

			Main.getMain().reset();
		}			
		catch (IOException e) {
			System.out.println(e.getMessage());
		}
	}

	public void addContactRenderControl () {
		ControlPanel panel = new ControlPanel ("options");

		CollisionManager cm = mechModel.getCollisionManager();      
		//		cm.setPenetrationTol(C.PENET_TOLERANCE_DEFAULT);
		//		cm.setCollisionPointTol(C.COL_POINT_TOL_DEFAULT);

		cm.setContactNormalLen (0);

		panel.addWidget ("Coulomb Coeff (mu)", this, "columbusCoeff");
		panel.addWidget ("PenetrationTol", this, "penetrationTol");

		cm.setDrawIntersectionContours (true);
		cm.setDrawIntersectionFaces (true);
		cm.setDrawIntersectionPoints (false);

		RenderProps.setVisible (cm, true);
		RenderProps.setLineColor (cm, Color.BLUE);
		RenderProps.setFaceColor (cm, Color.BLUE);

		panel.addWidget (mechModel, "integrator");
		panel.addWidget (cm, "collisionRegionTol");      
		panel.addWidget (cm, "contactNormalLen");
		panel.addWidget (cm, "penetrationTol");
		panel.addWidget (cm, "collisionPointTol");
		panel.addWidget (cm, "collisionAccel");
		panel.addWidget (cm, "collisionCompliance");
		panel.addWidget (cm, "collisionDamping");
		panel.addWidget ("contactsVisible", cm, "renderProps.visible");
		panel.addWidget ("normalWidth", cm, "renderProps.lineWidth");
		panel.addWidget ("normalColor", cm, "renderProps.lineColor");
		panel.addWidget ("contourWidth", cm, "renderProps.edgeWidth");
		panel.addWidget ("contourColor", cm, "renderProps.edgeColor");      
		addControlPanel (panel);
		Main.getMain ().arrangeControlPanels (this);
	}

	//	public StepAdjustment advance (double t0, double t1, int flags) {
	//		StepAdjustment adj = super.advance (t0, t1, flags);
	//		mechModel.advance (t0, t1, flags);
	//		return adj;
	//	}
}


//      mechModel.particles ().get (0).getPosition (pos);
//			mechModel.setDefaultCollisionBehavior(true, mu);