#include "addMeniscusToModel.h"

void addMeniscusToKnee(Model& model, bool LeftOrRight){
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	// // create body instance of tibia
	OpenSim::Body* tibia = &model.updBodySet().get("tibia_" + LorR);

	// create meniscus bodies
	double lat_meniscus_mass = 0.1;
	double med_meniscus_mass = 0.1;
	OpenSim::Body* lat_meniscus_body =  new OpenSim::Body("lat_meniscus_" + LorR, lat_meniscus_mass, SimTK::Vec3(0), 
		lat_meniscus_mass*SimTK::Inertia(1));
	OpenSim::Body* med_meniscus_body =  new OpenSim::Body("med_meniscus_" + LorR, med_meniscus_mass, SimTK::Vec3(0), 
		med_meniscus_mass*SimTK::Inertia(0));

	// Create meniscus display geometries
	DisplayGeometry * Lat_Meniscus_dg = new DisplayGeometry("Lat_Meniscus_"+LorR+".obj");
	Lat_Meniscus_dg->setOpacity(1.0);
	Lat_Meniscus_dg->setColor(Vec3(1.0, 0.5, 0.5));
	Lat_Meniscus_dg->setScaleFactors(SimTK::Vec3(1.0));
	DisplayGeometry * Med_Meniscus_dg = new DisplayGeometry("Med_Meniscus_"+LorR+".obj");
	Med_Meniscus_dg->setOpacity(1.0);
	Med_Meniscus_dg->setColor(Vec3(0.5, 0.7, 0.5));
	Med_Meniscus_dg->setScaleFactors(SimTK::Vec3(1.0));
	
	// Add meniscus display geometries to bodies
	lat_meniscus_body->updDisplayer()->updGeometrySet().cloneAndAppend(*Lat_Meniscus_dg);
	med_meniscus_body->updDisplayer()->updGeometrySet().cloneAndAppend(*Med_Meniscus_dg);

	// create meniscus - tibia pin jointS
	WeldJoint *latMenicscus_tibia_j = new WeldJoint::WeldJoint("latMeniscus_tibia_joint_" + LorR, *tibia, SimTK::Vec3(0.0), 
		SimTK::Vec3(0), *lat_meniscus_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 
	WeldJoint *medMenicscus_tibia_j = new WeldJoint::WeldJoint("medMeniscus_tibia_joint_" + LorR, *tibia, SimTK::Vec3(0.0), 
		SimTK::Vec3(0),	*med_meniscus_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 

	// add meniscus bodies to the model
	model.addBody(lat_meniscus_body);
	model.addBody(med_meniscus_body);	
}