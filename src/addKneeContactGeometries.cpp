#include "addKneeContactGeometries.h"

void addKneeContactGeometries(Model& model, bool LeftOrRight){
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	addContactGeometry(model, "lat_meniscus_r", "Lat_Meniscus_" + LorR + ".obj");
	addContactGeometry(model, "med_meniscus_r", "Med_Meniscus_" + LorR + ".obj");
	// addContactGeometry(model, "tibia_r", "Tibia_" + LorR + ".obj");
	addContactGeometry(model, "femur_r", "Femur_" + LorR + ".obj");
};

void addContactGeometry(Model& model, string bodyName, string objName){
	// create body instance of right lateral meniscus
	OpenSim::Body* body = &model.updBodySet().get(bodyName);

	// create contact mesh objects
	ContactMesh *contactMesh = new ContactMesh("/home/maria/Projects/ACLproj/resources/blender outputs in m/" + objName, 
		SimTK::Vec3(0.1), SimTK::Vec3(0.1), *body, bodyName + "_CM");	

	// Add contact mesh to the model
	model.addContactGeometry(contactMesh);
};