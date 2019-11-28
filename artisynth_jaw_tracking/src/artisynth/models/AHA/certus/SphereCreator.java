package artisynth.models.AHA.certus;

import artisynth.core.workspace.RootModel;
import artisynth.models.AHA.certus.*;

import javax.swing.event.MouseInputAdapter;

import java.awt.Color;
import java.awt.Component;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Vector;

import artisynth.core.driver.ViewerManager;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import maspack.matrix.Line;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.MouseRayEvent;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.GL.GLViewer;
import maspack.geometry.BVIntersector;
import maspack.geometry.BVTree;
import maspack.geometry.TriLineIntersection;

import java.util.Random;

public class SphereCreator extends MouseInputAdapter
{      
	BVTree bvtree;
	BVIntersector bvIntersector;
	RootModel rootModel;
	RigidBody mainRigidBody;
	MechModel mechModel;
	ViewerManager viewerManager;
	
	SphereCreator(ViewerManager viewerManager)
	{
		bvIntersector = new BVIntersector ();
		this.viewerManager = viewerManager;
	}

	public void set(MechModel mechModel, RigidBody rigidBody, RootModel rootModel)
	{
		this.bvtree = rigidBody.getMesh ().getBVTree ();
		this.mechModel = mechModel;
		this.rootModel = rootModel;     
		this.mainRigidBody = rigidBody;
	}

	public void mouseClicked(MouseEvent e)
	{
		//      System.out.println("mouse clicked");
		GLViewer viewer = ViewerManager.getViewerFromComponent ((Component)e.getSource());
		handleRayEvent(viewer, MouseRayEvent.create (e, viewer));
		if (Config.FLE_TEST)
			rotateViewerRandom(viewer);
	}

	private void rotateViewerRandom(GLViewer viewer) 
	{
		//	   RigidTransform3d viewmat = viewer.getViewMatrix();
		Point3d eye = viewer.getEye();
		Random r = new Random();	   
		float scale  = 50;
		eye.add(new Vector3d((r.nextFloat()-0.5)*scale, 
				(r.nextFloat()-0.5)*scale, 
				0));//, r.nextFloat()*scale));
		viewer.setEye(eye);
	}

	//private LinkedList<ModelComponent> myCopyList;
	//   private static final double inf = Double.POSITIVE_INFINITY;
	//   int sph_counter = 0;

	public void handleRayEvent (GLViewer viewer, MouseRayEvent rayEvent) {
		//      System.out.println (rayEvent);
		Line line = rayEvent.getRay ();
		//      System.out.println(line);
		Point3d eye = ((RootModel)rootModel).getMainViewer ().getEye ();

		ArrayList<TriLineIntersection> intersections = new ArrayList<> ();
		if (mainRigidBody.isSelected() && bvIntersector.intersectMeshLine
				(intersections, bvtree, line))
		{
			int numPoints = intersections.size ();
			//         System.out.println ("ray went through num_points: " + numPoints);         
			Point3d[] points = new Point3d[numPoints];
			for (int i=0; i<numPoints; ++i) {         
				points[i] = intersections.get (i).points[0];
			}


			Point3d closestPointToEye = points[0];
			for (int i = 1; i<numPoints; ++i) {         
				if (points[i].distance (eye) < closestPointToEye.distance (eye))
					closestPointToEye = points[i];            
			}                    
			Point3d castPoint = new Point3d(closestPointToEye);

			if (Config.SPHERE_METHOD)
			{
				Vector3d vector2Eye = eye.clone();
				vector2Eye.sub(closestPointToEye);
				System.out.println(closestPointToEye);

				vector2Eye.scale(0.3);
				Point3d sphereStartPoint = new Point3d(closestPointToEye);
				sphereStartPoint.add(vector2Eye);

				int sph_counter = utils.countSpheres(mechModel);

				RigidBody sph = utils.add_sphere (mechModel, String.format ("sph%1$1s_body", sph_counter), 
						Config.STYLUS_HEAD_Radius, 100, 100, sphereStartPoint, Color.blue);

				Particle particleSph = new Particle(0.1, sphereStartPoint);
				Particle particleCast = new Particle(0.1, castPoint);
				mechModel.addParticle(particleCast);
				mechModel.addParticle(particleSph);

				AxialSpring spring = new AxialSpring(0,0,0);//String.format ("spring%1$1s", sph_counter), 0);
				spring.setPoints(particleCast, particleSph);   
				spring.setMaterial(new LinearAxialMaterial(0.1, 0.1));                                 
				mechModel.addAxialSpring(spring);
				//	         RenderProps.setVisible(spring, false);

				mechModel.attachPoint(particleSph, sph);
				mechModel.attachPoint(particleCast, mainRigidBody);
				particleCast.setDynamic(false);
				particleSph.setDynamic(true);

				RenderProps.setSphericalPoints (particleSph, 0.06, Color.RED);
				RenderProps.setSphericalPoints (particleCast, 0.06, Color.RED);
				RenderProps.setVisible(particleCast, false);
				RenderProps.setVisible(particleSph, false);

				RenderProps.setCylindricalLines (spring, 0.02, Color.BLUE);

				mechModel.setCollisionBehavior (sph, mainRigidBody, true);
			}
			else
			{
				System.out.println(castPoint);
			}
		}
		viewerManager.render();
	}



}       