package artisynth.models.AHA.certus;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.compress.compressors.FileNameUtil;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.models.AHA.certus.*;
//import artisynth.models.frank2.GenericModel;
import maspack.geometry.io.*; 
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.PlyReader;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyMode;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;
import sun.security.jgss.spi.MechanismFactory;
import org.apache.commons.*;

public class utils
{   
   private utils() {}

   static public RigidBody add_cast(MechModel mech, String name, 
      String meshFilename, double density)
   {
	   System.out.println("Working Directory = " +
	              System.getProperty("user.dir"));
	   
      RigidBody body = new RigidBody (name);
      System.out.println ("loading mesh from: "+ meshFilename);
      PolygonalMesh mesh = null;      
      String ext = meshFilename.substring(meshFilename.length()-3, meshFilename.length());
      
      try {		         
    	  StlReader reader = new StlReader(Config.data_path + meshFilename);
//    	  mesh = GenericModel.loadGeometry(Config.data_path, meshFilename);
    	  mesh = reader.readMesh();
//          mesh = new PolygonalMesh (new File (meshFilename));
       }
       catch (Exception e) {
          e.printStackTrace();
          return null;
       }
      
//      mesh.scale (0.001); // convert mesh from mm to m
      body.setMesh (mesh, meshFilename);
      System.out.println ("mesh loaded.");
      body.setDynamic (false);
      RenderProps.setFaceColor (body, Color.WHITE);
      RenderProps.setFaceStyle (body, FaceStyle.FRONT_AND_BACK);
      body.setInertiaFromDensity (density*1e-9); //rescale density of kg/mm^3
      mech.addRigidBody (body);
      
      return body;
   }

   public static RigidBody add_sphere (MechModel mechModel, String name, 
      double radius, int slices, int levels, Vector3d pos, Color color) 
   {
      RigidBody sph_body = new RigidBody (name);
      PolygonalMesh sph = MeshFactory.createSphere (radius, slices, levels);      
      sph.setColorsFixed (true);

      sph_body.setMesh (sph);
      sph_body.setDensity(Config.SPHERE_DENSITY*1e-9);
      
      mechModel.addRigidBody (sph_body);
      RenderProps.setFaceColor (sph_body, color);
//      RenderProps.setAlpha(sph_body, 0.95);
//      RenderProps.setAlphaMode(sph_body, PropertyMode.Explicit);
//      RenderProps.setShading(sph_body, Shading.SMOOTH);
//      RenderProps.setShininess(sph_body, 50);
//      RenderProps.setTextureEnabled(sph_body, true);      
      
      sph_body.setPosition (new Point3d(pos));
      System.out.println ("sphere " + name + " created at position " + sph_body.getPosition ().toString ());
      return sph_body;
   }
   
   public static void transformtoLocal (RigidBody rigidBody) {
    Point3d pointVector = rigidBody.getCenterOfMass ();
    pointVector.negate ();
    rigidBody.getMesh ().translate (pointVector);
    pointVector.negate ();
    rigidBody.setPosition (pointVector);
    
    Point3d testPos = rigidBody.getPosition ();
    System.out.println (rigidBody.getName () + " transformed to " + testPos.toString ());
   }

   public static void transformtoLocal (RigidBody rigidBody, Point3d pointVector) { 
      pointVector.negate ();
      rigidBody.getMesh ().translate (pointVector);
      
      pointVector.negate ();
      rigidBody.setPosition (pointVector);
      
      Point3d testPos = rigidBody.getPosition ();
      System.out.println (rigidBody.getName () + " transformed to " + testPos.toString ());
      
   }
   
   public static void transformMeshtoLocal (RigidBody rigidBody, Point3d pointVector) { 
	      pointVector.negate ();
	      rigidBody.getMesh ().translate (pointVector);
	      Point3d testPos = rigidBody.getPosition ();
	      System.out.println (rigidBody.getName () + " transformed to " + testPos.toString ());
   }

   public static int countSpheres(MechModel mechModel)
   {
	   int sph_counter = 0;
		
		while (true)
		{
			RigidBody sph = mechModel.rigidBodies ().
					get (String.format ("sph%1$1s_body", sph_counter++));
			if (sph == null)
			{
				sph_counter--;
				break;
			}
		}
		return sph_counter;		
   }
   
   public static void transformMeshOrigin(PolygonalMesh mesh)
   {
	   ArrayList<Vertex3d> verts = mesh.getVertices();
	   Point3d com = new Point3d();
	   
	   for (Vertex3d vertex3d : verts) {
		com.add(vertex3d.getPosition());		
	   }
	   com.scale(1.0/verts.size());
	   com.negate();
	   mesh.translate(com);
	   
   }
   
}