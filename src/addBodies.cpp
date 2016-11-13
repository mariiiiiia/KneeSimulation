#include "addBodies.h"

void addMeniscusWeldJoints(Model& model, bool LeftOrRight){
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	// create body instance of tibia
	OpenSim::Body* tibia = &model.updBodySet().get("tibia_" + LorR);

	// create meniscus bodies
	double lat_meniscus_mass = 0.1;
	double med_meniscus_mass = 0.1;
	OpenSim::Body* lat_meniscus_body =  new OpenSim::Body("meniscus_lat_" + LorR, lat_meniscus_mass, SimTK::Vec3(0), 
		lat_meniscus_mass*SimTK::Inertia(0.0));
	OpenSim::Body* med_meniscus_body =  new OpenSim::Body("meniscus_med_" + LorR, med_meniscus_mass, SimTK::Vec3(0), 
		med_meniscus_mass*SimTK::Inertia(0.0));

	// Create meniscus display geometries
	DisplayGeometry * Lat_Meniscus_dg = new DisplayGeometry("meniscus_lat_"+LorR+".obj");
	Lat_Meniscus_dg->setOpacity(1.0);
	Lat_Meniscus_dg->setColor(Vec3(1.0, 0.5, 0.5));
	Lat_Meniscus_dg->setScaleFactors(SimTK::Vec3(1.0));
	DisplayGeometry * Med_Meniscus_dg = new DisplayGeometry("meniscus_med_"+LorR+".obj");
	Med_Meniscus_dg->setOpacity(1.0);
	Med_Meniscus_dg->setColor(Vec3(0.5, 0.7, 0.5));
	Med_Meniscus_dg->setScaleFactors(SimTK::Vec3(1.0));
	
	// Add meniscus display geometries to bodies
	lat_meniscus_body->updDisplayer()->updGeometrySet().cloneAndAppend(*Lat_Meniscus_dg);
	med_meniscus_body->updDisplayer()->updGeometrySet().cloneAndAppend(*Med_Meniscus_dg);

	// create meniscus - tibia pin jointS
	WeldJoint *latMenicscus_tibia_j = new WeldJoint("latMeniscus_tibia_joint_" + LorR, *tibia, SimTK::Vec3(0.0), 
		SimTK::Vec3(0), *lat_meniscus_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 
	WeldJoint *medMenicscus_tibia_j = new WeldJoint("medMeniscus_tibia_joint_" + LorR, *tibia, SimTK::Vec3(0.0), 
		SimTK::Vec3(0),	*med_meniscus_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 

	// add meniscus bodies to the model
	model.addBody(lat_meniscus_body);
	model.addBody(med_meniscus_body);	
}

void addFemurWeldJoints(Model& model, bool LeftOrRight)
{
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	// create body instance of femur
	OpenSim::Body* femur = &model.updBodySet().get("femur_" + LorR);

	// create femur lateral and medial bodies
	double lat_femur_mass = 0.1;
	double med_femur_mass = 0.1;
	OpenSim::Body* lat_femur_body =  new OpenSim::Body("femur_lat_" + LorR, lat_femur_mass, SimTK::Vec3(0), 
		lat_femur_mass*SimTK::Inertia(0.0));
	OpenSim::Body* med_femur_body =  new OpenSim::Body("femur_med_" + LorR, med_femur_mass, SimTK::Vec3(0), 
		med_femur_mass*SimTK::Inertia(0.0));

	// Create meniscus display geometries
	DisplayGeometry * Lat_Femur_dg = new DisplayGeometry("femur_lat_"+LorR+".obj");
	Lat_Femur_dg->setOpacity(1.0);
	Lat_Femur_dg->setColor(Vec3(0.3, 0.8, 0.9));
	Lat_Femur_dg->setScaleFactors(SimTK::Vec3(1.0));
	DisplayGeometry * Med_Femur_dg = new DisplayGeometry("femur_med_"+LorR+".obj");
	Med_Femur_dg->setOpacity(1.0);
	Med_Femur_dg->setColor(Vec3(0.5, 0.7, 0.2));
	Med_Femur_dg->setScaleFactors(SimTK::Vec3(1.0));
	
	// Add meniscus display geometries to bodies
	lat_femur_body->updDisplayer()->updGeometrySet().cloneAndAppend(*Lat_Femur_dg);
	med_femur_body->updDisplayer()->updGeometrySet().cloneAndAppend(*Med_Femur_dg);

	// create femur-femur_lat and femur-femur_med weld joints
	WeldJoint *latFemur_j = new WeldJoint("latFemur_joint_" + LorR, *femur, SimTK::Vec3(0.0), 
		SimTK::Vec3(0), *lat_femur_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 
	WeldJoint *medFemur_j = new WeldJoint("medFemur_joint_" + LorR, *femur, SimTK::Vec3(0.0), 
		SimTK::Vec3(0),	*med_femur_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 

	// add meniscus bodies to the model
	model.addBody(lat_femur_body);
	model.addBody(med_femur_body);	
}

void addTibiaWeldJoints(Model& model, bool LeftOrRight)
{
	string LorR = "r";
	if (LeftOrRight) LorR = "l";

	// create body instance of tibia
	OpenSim::Body* tibia = &model.updBodySet().get("tibia_" + LorR);

	// create upper tibia bodies
	double upper_tibia_mass = 0.1;
	OpenSim::Body* upper_tibia_body =  new OpenSim::Body("tibia_upper_" + LorR, upper_tibia_mass, SimTK::Vec3(0), 
		upper_tibia_mass*SimTK::Inertia(0.0));

	// Create meniscus display geometries
	DisplayGeometry * upper_tibia_dg = new DisplayGeometry("tibia_upper_"+LorR+".obj");
	upper_tibia_dg->setOpacity(1.0);
	upper_tibia_dg->setColor(Vec3(0.6, 0.4, 0.9));
	upper_tibia_dg->setScaleFactors(SimTK::Vec3(1.0));
	
	// Add meniscus display geometries to bodies
	upper_tibia_body->updDisplayer()->updGeometrySet().cloneAndAppend(*upper_tibia_dg);

	// create meniscus - tibia pin jointS
	WeldJoint *upper_tibia_j = new WeldJoint("tibia_weldJoint_" + LorR, *tibia, SimTK::Vec3(0.0), 
		SimTK::Vec3(0), *upper_tibia_body, SimTK::Vec3(0), SimTK::Vec3(0), true); 

	// add meniscus bodies to the model
	model.addBody(upper_tibia_body);
}

