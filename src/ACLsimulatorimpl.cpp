#include "ACLsimulatorimpl.h"
#include "osimutils.h"
#include <ctime>
#include "CustomLigament.h"
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

Array_<State> saveEm;

static const Real TimeScale = 1;
static const Real FrameRate = 30;
static const Real ReportInterval = TimeScale/FrameRate;

// These are the item numbers for the entries on the Run menu.
static const int RunMenuId = 3, HelpMenuId = 7;
static const int GoItem = 1, ReplayItem=2, QuitItem=3;

template<typename T> std::string changeToString(
    const T& value, int precision = std::numeric_limits<int>::infinity())
{
    std::ostringstream oss;
    if (precision != std::numeric_limits<int>::infinity())
    {
        oss << std::setprecision(precision);
    }     oss << value;     return oss.str();
}

class HitMapGenerator : public DecorationGenerator {
public:
    HitMapGenerator(const MultibodySystem& system,
                        const CompliantContactSubsystem& complCont) 
    :   m_system(system), m_compliant(complCont){}

    virtual void generateDecorations(const State& state, Array_<DecorativeGeometry>& geometry) override {
        const Vec3 frcColors[] = {Red,Orange,Cyan,Green,Blue,Yellow};
        m_system.realize(state, Stage::Velocity);

        const int ncont = m_compliant.getNumContactForces(state);
        for (int i=0; i < ncont; ++i) {
            const ContactForce& force = m_compliant.getContactForce(state,i);
            const ContactId     id    = force.getContactId();
			// define if it's femur_lat -> femur_med contact pair: id=8
			//if (id == 6 || id == 8 || id==4 || id==2 || id==10 )
			//	continue;
			ContactSnapshot cs = m_compliant.getContactTrackerSubsystem().getActiveContacts(state);
			//cout <<  "contact: " << id << endl;
			//ContactId idFemur = cs.getContactIdForSurfacePair( ContactSurfaceIndex(2), ContactSurfaceIndex(3));
			//ContactId idMeniscFemur1 = cs.getContactIdForSurfacePair( ContactSurfaceIndex(0), ContactSurfaceIndex(2));
			//ContactId idMeniscTibia1 = cs.getContactIdForSurfacePair( ContactSurfaceIndex(0), ContactSurfaceIndex(4));
			//ContactId idMeniscFemur2 = cs.getContactIdForSurfacePair( ContactSurfaceIndex(1), ContactSurfaceIndex(3));
			//ContactId idMeniscTibia2 = cs.getContactIdForSurfacePair( ContactSurfaceIndex(1), ContactSurfaceIndex(4));
			ContactId idLatFemurTibia = cs.getContactIdForSurfacePair( ContactSurfaceIndex(0), ContactSurfaceIndex(2));
			ContactId idMedFemurTibia = cs.getContactIdForSurfacePair( ContactSurfaceIndex(1), ContactSurfaceIndex(2));
			ContactId idFemur = cs.getContactIdForSurfacePair( ContactSurfaceIndex(0), ContactSurfaceIndex(1));

			if (id == idFemur)
				continue;

			const SimbodyMatterSubsystem& matter = m_compliant.getMultibodySystem().getMatterSubsystem();
			// get tibia's mobilized body
			MobilizedBody mobod = matter.getMobilizedBody( MobilizedBodyIndex(22));
			//for (int numContacts=0; numContacts<cs.getNumContacts(); numContacts++)
			//{
				//ContactSurfaceIndex csi1 = cs.getContact(numContacts).getSurface1();
				//ContactSurfaceIndex csi2 = cs.getContact(numContacts).getSurface2();
				//cout << "surf1: " << csi1 << " surf2: " << csi2 << endl;
			//}
			if (cs.getNumContacts()>0)
			{
				DecorativeGeometry decContactGeometry = mobod.getBody().updDecoration(0);
				decContactGeometry.setOpacity( 0.05);
				decContactGeometry.setColor( Gray);
				//decContactGeometry.setRepresentation( DecorativeGeometry::Representation::DrawPoints);

				const Transform& X_BM = mobod.getOutboardFrame(state); // M frame in B
				const Transform& X_GB = mobod.getBodyTransform(state); // B in Ground
				const Transform& X_PF = mobod.getInboardFrame(state);
				Transform X_GM = X_GB*X_BM; // M frame in Ground
				//cout << "location of Mo in Ground: " << X_GM.p() << endl;
				// rotate 
				Rotation newRot = X_GM.updR();
				Rotation parentRot = X_PF.R();
				newRot.operator=( newRot.operator*=( parentRot));
				// translate in front of the whole body
				X_GM.setP( X_GM.operator+=( Vec3(0.5,0,0)).p());
				// set new transform
				decContactGeometry.setTransform( X_GM);

				ContactPatch patch;
				const bool found = m_compliant.calcContactPatchDetailsById(state,id,patch);
				//cout << "patch for id" << id << " found=" << found << endl;
				//cout << "resultant=" << patch.getContactForce() << endl;
				//cout << "num details=" << patch.getNumDetails() << endl;
				for (int k=0; k < patch.getNumDetails(); ++k) {
					const ContactDetail& detail = patch.getContactDetail(k);
					//const Real peakPressure = detail.getPeakPressure();			
					const Vec3 cp = detail.getContactPoint();
					Vec3 newcp = Vec3(cp);
					Transform newTransf = Transform(newcp);
					newTransf.setP( newTransf.operator+=( Vec3(0.5,0,0)).p());

					DecorativeSphere decContP;
					decContP.setScale( 0.0005);
					decContP.setTransform( newTransf);
					decContP.setColor(Red);
					geometry.push_back(decContP);
					geometry.push_back( decContactGeometry);
				}
			}
        }		
    }
private:
    const MultibodySystem&              m_system;
    const CompliantContactSubsystem&    m_compliant;
};

class MyReporter : public PeriodicEventReporter {
public:
    MyReporter(const MultibodySystem& system, 
               const CompliantContactSubsystem& complCont,
               Real reportInterval)
    :   PeriodicEventReporter(reportInterval), m_system(system),
        m_compliant(complCont)
    {}

    ~MyReporter() {}

    void handleEvent(const State& state) const override {
        m_system.realize(state, Stage::Dynamics);
        cout << state.getTime() << ": E = " << m_system.calcEnergy(state)
             << " Ediss=" << m_compliant.getDissipatedEnergy(state)
             << " E+Ediss=" << m_system.calcEnergy(state)
                               +m_compliant.getDissipatedEnergy(state)
             << endl;
        const int ncont = m_compliant.getNumContactForces(state);
        //cout << "Num contacts: " << ncont << endl;
        for (int i=0; i < ncont; ++i) {
            const ContactForce& force = m_compliant.getContactForce(state,i);
            //cout << force;
        }
        saveEm.push_back(state);
    }
private:
    const MultibodySystem&           m_system;
    const CompliantContactSubsystem& m_compliant;
};

// This is a periodic event handler that interrupts the simulation on a regular
// basis to poll the InputSilo for user input. If there has been some, process it.
// This one does nothing but look for the Run->Quit selection.
class UserInputHandler : public PeriodicEventHandler {
public:
    UserInputHandler(Visualizer::InputSilo& silo, Real interval) 
    :   PeriodicEventHandler(interval), m_silo(silo) {};

    virtual void handleEvent(State& state, Real accuracy, 
                             bool& shouldTerminate) const override 
    {
        int menuId, item;
		//cout << "hello " << m_silo.isAnyUserInput() << " " << menuId << " " << item;
        if (m_silo.takeMenuPick(menuId, item) && item==QuitItem)
            shouldTerminate = true;
		//while (m_silo.isAnyUserInput()) {
		//	while (silo.takeKeyHit(key,modifier)) {
		//		 Process the key that was hit
		//		cout << "Key: " << key << endl;
		//	}
		//	while (m_silo.takeMenuPick(menuId, item)) {
		//		 Process the picked menu item
		//		cout << "Menu item: " << item << endl;
		//	}
		//	while (silo.takeSliderMove(which, value)) {
		//		 Process the new value for slider "which"
		//		cout << "Slider: " << which << endl;
		//	}
		//}
	}

private:
    Visualizer::InputSilo& m_silo;
};

void anteriorTibialLoadsFD(Model& model, double knee_angle)
{
	addTibialLoads(model, knee_angle);
	
	// init system
	std::time_t result = std::time(nullptr);
	std::cout << "\nBefore initSystem() " << std::asctime(std::localtime(&result)) << endl;
	SimTK::State& si = model.initSystem();
	result = std::time(nullptr);
	std::cout << "\nAfter initSystem() " << std::asctime(std::localtime(&result)) << endl;
	
	// set gravity	
	model.updGravityForce().setGravityVector(si, Vec3(0,0,0));

	setKneeAngle(model, si, knee_angle, true, true);
	//setATT(model,si, knee_angle);
	// disable muscles
	//string muscle_name;
	//for (int i=0; i<model.getActuators().getSize(); i++)
	//{
	//	muscle_name = model.getActuators().get(i).getName();

	//	model.getActuators().get(i).setDisabled(si, true);

		//if (muscle_name == "bifemlh_r" || muscle_name == "bifemsh_r" || muscle_name == "grac_r" \
		//	|| muscle_name == "lat_gas_r" || muscle_name == "med_gas_r" || muscle_name == "sar_r" \
		//	|| muscle_name == "semimem_r" || muscle_name == "semiten_r" \
		//	|| muscle_name == "rect_fem_r" || muscle_name == "vas_med_r" || muscle_name == "vas_int_r" || muscle_name == "vas_lat_r" )
		//		model.getActuators().get(i).setDisabled(si, false);
	//}
	model.equilibrateMuscles( si);

	// Add reporters
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

	CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
	model.addAnalysis(customReporter);

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	//SimTK::CPodesIntegrator integrator(model.getMultibodySystem());
	//integrator.setAccuracy(1.0e-3);
	//integrator.setFixedStepSize(0.001);
	Manager manager(model, integrator);

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 1.0;

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<std::endl;

	result = std::time(nullptr);
	std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	manager.integrate(si);

	result = std::time(nullptr);
	std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	// Save the simulation results
	//osimModel.updAnalysisSet().adoptAndAppend(forces);
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("../outputs/states_ant_load_" + changeToString(abs(knee_angle)) +".sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("../outputs/states_degrees_ant_load_" + changeToString(abs(knee_angle)) +".mot");
	// force reporter results
	model.updAnalysisSet().adoptAndAppend(forceReporter);
	forceReporter->getForceStorage().print("../outputs/force_reporter_ant_load_" + changeToString(abs(knee_angle)) +".mot");
	customReporter->print( "../outputs/custom_reporter_ant_load_" + changeToString(abs(knee_angle)) +".mot");

	model.removeAnalysis(forceReporter);
	model.removeAnalysis(customReporter);
}

void flexionFDSimulation(Model& model)
{
	addFlexionController(model);
	//addExtensionController(model);

	// init system
	std::time_t result = std::time(nullptr);
	std::cout << "\nBefore initSystem() " << std::asctime(std::localtime(&result)) << endl;
	SimTK::State& si = model.initSystem();
	result = std::time(nullptr);
	std::cout << "\nAfter initSystem() " << std::asctime(std::localtime(&result)) << endl;
	
	// set gravity
	model.updGravityForce().setGravityVector(si, Vec3(-9.80665,0,0));
	//model.updGravityForce().setGravityVector(si, Vec3(0,0,0));
	
	setHipAngle(model, si, 90);
	setKneeAngle(model, si, 0, false, false);
	model.equilibrateMuscles( si);

	// Add reporters
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

	CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
	model.addAnalysis(customReporter);

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	//SimTK::CPodesIntegrator integrator(model.getMultibodySystem());
	//integrator.setAccuracy(.01);
	//integrator.setAccuracy(1e-3);
	//integrator.setFixedStepSize(0.001);
	Manager manager(model, integrator);

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 0.2;

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<std::endl;

	result = std::time(nullptr);
	std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	manager.integrate(si);

	result = std::time(nullptr);
	std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("../outputs/states_flex.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("../outputs/states_degrees_flex.mot");
	// force reporter results
	forceReporter->getForceStorage().print("../outputs/force_reporter_flex.mot");
	customReporter->print( "../outputs/custom_reporter_flex.mot");
}

void flexionFDSimulationWithHitMap(Model& model)
{
	addFlexionController(model);
	//addExtensionController(model);
	//addTibialLoads(model, 0);

    model.setUseVisualizer(true);

	// init system
	std::time_t result = std::time(nullptr);
	std::cout << "\nBefore initSystem() " << std::asctime(std::localtime(&result)) << endl;
	SimTK::State& si = model.initSystem();
	result = std::time(nullptr);
	std::cout << "\nAfter initSystem() " << std::asctime(std::localtime(&result)) << endl;
	
	// set gravity
	//model.updGravityForce().setGravityVector(si, Vec3(-9.80665,0,0));
	//model.updGravityForce().setGravityVector(si, Vec3(0,0,0));
	
	//setHipAngle(model, si, 90);
	//setKneeAngle(model, si, 0, false, false);
	//model.equilibrateMuscles( si);

	MultibodySystem& system = model.updMultibodySystem();
	SimbodyMatterSubsystem& matter = system.updMatterSubsystem();
	GeneralForceSubsystem forces(system);
	ContactTrackerSubsystem  tracker(system);
    CompliantContactSubsystem contactForces(system, tracker);
	contactForces.setTrackDissipatedEnergy(true);
    //contactForces.setTransitionVelocity(1e-3);

	for (int i=0; i < matter.getNumBodies(); ++i) {
		MobilizedBodyIndex mbx(i);
		if (i==19 || i==20 || i==22)// || i==15 || i==16)
		{
			MobilizedBody& mobod = matter.updMobilizedBody(mbx);
			std::filebuf fb;
			//cout << mobod.updBody().
			if (i==19)
				fb.open ( "../resources/femur_lat_r.obj",std::ios::in);
			else if (i==20)
				fb.open ( "../resources/femur_med_r.obj",std::ios::in);
			else if (i==22)
				fb.open ( "../resources/tibia_upper_r.obj",std::ios::in);
			//else if (i==15)
				//fb.open ( "../resources/meniscus_lat_r.obj",std::ios::in);
			//else if (i==16)
				//fb.open ( "../resources/meniscus_med_r.obj",std::ios::in);
			std::istream is(&fb);
			PolygonalMesh polMesh;
			polMesh.loadObjFile(is);
			fb.close();
			SimTK::ContactGeometry::TriangleMesh mesh(polMesh);
			ContactSurface contSurf;//(mesh, ContactMaterial(1.0e6, 1, 1, 0.03, 0.03), 0.001);
			if (i==19 || i==20 || i==22)
				contSurf = ContactSurface(mesh, ContactMaterial(10, 1, 1, 0.03, 0.03), 0.001);
			//else if (i==15 || i==16)
				//contSurf = ContactSurface(mesh, ContactMaterial(10, 3, 1, 0.03, 0.03), 0.001);
			DecorativeMesh showMesh(mesh.createPolygonalMesh());
			showMesh.setOpacity(0.5);
			mobod.updBody().addDecoration( showMesh);
			mobod.updBody().addContactSurface(contSurf);
		}
    }

	ModelVisualizer& viz(model.updVisualizer());
	//Visualizer viz(system);
	viz.updSimbodyVisualizer().addDecorationGenerator(new HitMapGenerator(system,contactForces));
    viz.updSimbodyVisualizer().setMode(Visualizer::RealTime);
    viz.updSimbodyVisualizer().setDesiredBufferLengthInSec(1);
    viz.updSimbodyVisualizer().setDesiredFrameRate(30);
    viz.updSimbodyVisualizer().setGroundHeight(-3);
    viz.updSimbodyVisualizer().setShowShadows(true);
	
    Visualizer::InputSilo* silo = new Visualizer::InputSilo();
	viz.updSimbodyVisualizer().addInputListener(silo);
    Array_<std::pair<String,int> > runMenuItems;
    runMenuItems.push_back(std::make_pair("Go", GoItem));
    runMenuItems.push_back(std::make_pair("Replay", ReplayItem));
    runMenuItems.push_back(std::make_pair("Quit", QuitItem));
    viz.updSimbodyVisualizer().addMenu("Run", RunMenuId, runMenuItems);

    Array_<std::pair<String,int> > helpMenuItems;
    helpMenuItems.push_back(std::make_pair("TBD - Sorry!", 1));
    viz.updSimbodyVisualizer().addMenu("Help", HelpMenuId, helpMenuItems);

    system.addEventReporter(new MyReporter(system,contactForces,ReportInterval));
	//system.addEventReporter(new Visualizer::Reporter(viz.updSimbodyVisualizer(), ReportInterval));
	
    // Check for a Run->Quit menu pick every <x> second.
	system.addEventHandler(new UserInputHandler(*silo, 0.001));

	system.realizeTopology();

	//Show ContactSurfaceIndex for each contact surface
  //  for (int i=0; i < matter.getNumBodies(); ++i) {
		//MobilizedBodyIndex mbx(i);
  //      const MobilizedBody& mobod = matter.getMobilizedBody(mbx);
  //      const int nsurfs = mobod.getBody().getNumContactSurfaces();
        //printf("mobod %d has %d contact surfaces\n", (int)mbx, nsurfs);
		////cout << "mobod with mass: " << (float)mobod.getBodyMass(si) << " has " << nsurfs << " contact surfaces" << endl;
		// //for (int i=0; i<nsurfs; ++i) {
  //          //printf("%2d: index %d\n", i, 
  //                 //(int)tracker.getContactSurfaceIndex(mbx,i)); 
  //      //}
  //  }

	//cout << "tracker num of surfaces: " << tracker.getNumSurfaces() << endl;

	//State state = system.getDefaultState();
	//viz.report(state);
	State& state = model.initializeState();
	viz.updSimbodyVisualizer().report(state);

	//cout << "\nChoose 'Go' from Run menu to simulate:\n" << endl;
    int menuId, item;
    //do { silo->waitForMenuPick(menuId, item);
	//do { viz.updInputSilo().waitForMenuPick(menuId, item);
         //if (menuId != RunMenuId || item != GoItem) 
             //cout << "\aDude ... follow instructions!\n";
    //} while (menuId != RunMenuId || item != GoItem);

	// Add reporters
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

	CustomAnalysis* customReporter = new CustomAnalysis(&model, "r");
	model.addAnalysis(customReporter);

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	//SimTK::CPodesIntegrator integrator(model.getMultibodySystem());
	//integrator.setAccuracy(.01);
	//integrator.setAccuracy(1e-3);
	//integrator.setFixedStepSize(0.001);
	Manager manager(model, integrator);

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 0.1;

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to " <<finalTime<<std::endl;

	result = std::time(nullptr);
	std::cout << "\nBefore integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	manager.integrate(state);

	result = std::time(nullptr);
	std::cout << "\nAfter integrate(si) " << std::asctime(std::localtime(&result)) << endl;

	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("../outputs/states_flex.sto");
	model.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("../outputs/states_degrees_flex.mot");
	// force reporter results
	forceReporter->getForceStorage().print("../outputs/force_reporter_flex.mot");
	//customReporter->print( "../outputs/custom_reporter_flex.mot");

	//cout << "You can choose 'Replay'" << endl;
	unsigned int frameRate = 5;
	do { 
		cout << "Please choose 'Replay' or 'Quit'" << endl;
		viz.updInputSilo().waitForMenuPick(menuId, item);

        if (item != ReplayItem && item != QuitItem) 
            cout << "\aDude... follow instructions!\n";
		if (item == ReplayItem)
		{
			cout << "Type desired frame rate for playback and press Enter (default = 1) : ";
			//frameRate = cin.get();		
			cin >> frameRate;
			if (cin.fail()) 
			{
				cout << "Not an int. Setting default frame rate." << endl;
				cin.clear();
				cin.ignore(std::numeric_limits<int>::max(),'\n');
				frameRate = 1;
			}

			//cout << "saveEm size: " << saveEm.size() << endl;
			for (unsigned int i=0; i<saveEm.size(); i++)
			{
				viz.updSimbodyVisualizer().drawFrameNow(saveEm.getElt(i));
				usleep(1000000/frameRate);
			}
		}
	} while (menuId != RunMenuId || item != QuitItem);
}

void addTibialLoads(Model& model, double knee_angle)
{
	// edit prescribed force
	PrescribedForce& prescribedForce = dynamic_cast<PrescribedForce&>(model.updForceSet().get("prescribedForce"));
	// Create a new prescribed force applied to the block
	//PrescribedForce *prescribedForce = new PrescribedForce();
	//ostringstream strs;
	//strs << "prescribedForce_" << knee_angle;
	//prescribedForce->setName(strs.str());
	prescribedForce.setBodyName("tibia_r");

	// Set the force and point functions for the new prescribed force
	if (knee_angle == 0)
		prescribedForce.setForceFunctions(new Constant(110.0), new Constant(0.0), new Constant(0.0));		// at 0 degrees
	else if (knee_angle == -15)
		prescribedForce.setForceFunctions(new Constant(106.25), new Constant(-28.47), new Constant(0.0));	// at -15 degrees
	else if (knee_angle == -20)
		prescribedForce.setForceFunctions(new Constant(103.366188), new Constant(-37.6222), new Constant(0.0));	// at -20 degrees (knee_angle)
	else if (knee_angle == -30)
		prescribedForce.setForceFunctions(new Constant(95.26279), new Constant(-55), new Constant(0.0));	// at -20 degrees (knee_angle)
	else if (knee_angle == -40)
		prescribedForce.setForceFunctions(new Constant(84.26488), new Constant(-70.7066), new Constant(0.0));	// at -40 degrees (knee_angle)
	else if (knee_angle == -60)
		prescribedForce.setForceFunctions(new Constant(55), new Constant(-95.2627), new Constant(0.0));	// at -60 degrees (knee_angle)
	else if (knee_angle == -80)
		prescribedForce.setForceFunctions(new Constant(19.101), new Constant(-108.3288), new Constant(0.0));	// at -80 degrees (knee_angle)	
	else if (knee_angle == -90)
		prescribedForce.setForceFunctions(new Constant(0), new Constant(-110.0), new Constant(0.0));	// at -90 degrees (knee_angle)	

	prescribedForce.setPointFunctions(new Constant(0.03), new Constant(-0.03), new Constant(0));

	//prescribedForce->setForceIsInGlobalFrame(true);
	prescribedForce.setPointIsInGlobalFrame(false);

	// Add the new prescribed force to the model
	//model.addForce(prescribedForce);
}

void addFlexionController(Model& model)
{
	PrescribedController* controller = new PrescribedController();
	controller->setName( "flexion_controller");
	controller->setActuators( model.updActuators());
	
	double control_time[2] = {0, 0.05}; // time nodes for linear function
	double control_acts[2] = {1.0, 0}; // force values at t1 and t2

	string muscle_name;
	for (int i=0; i<model.getActuators().getSize(); i++)
	{
		muscle_name = model.getActuators().get(i).getName();
		// hamstrings: bi*, semi*
		if ( muscle_name == "bifemlh_r" || muscle_name == "bifemsh_r" || muscle_name == "grac_r" \
			|| muscle_name == "lat_gas_r" || muscle_name == "med_gas_r" || muscle_name == "sar_r" \
			|| muscle_name == "semimem_r" || muscle_name == "semiten_r")
		{
			Constant* ccf = new Constant(1.0);
			//PiecewiseLinearFunction *ccf = new PiecewiseLinearFunction( 2, control_time, control_acts);
			controller->prescribeControlForActuator( i, ccf);
		}
		else 
		{
			Constant* zccf = new Constant(0);
			controller->prescribeControlForActuator( i, zccf);
		}
	}
	model.addController( controller);
}

void addExtensionController(Model& model)
{
	PrescribedController* controller = new PrescribedController();
	controller->setName( "extension_controller");
	controller->setActuators( model.updActuators());
	
	double control_time[2] = {0.01, 0.02}; // time nodes for linear function
	double control_acts[2] = {0.0, 1.0}; // force values at t1 and t2
	//control_func->setName( "constant_control_func");

	string muscle_name;
	for (int i=0; i<model.getActuators().getSize(); i++)
	{
		muscle_name = model.getActuators().get(i).getName();
		// activate quadriceps
		if (muscle_name == "rect_fem_r" || muscle_name == "vas_med_r" || muscle_name == "vas_int_r" || muscle_name == "vas_lat_r" )
		{
			Constant* ccf = new Constant(0.8);
			//PiecewiseLinearFunction *ccf = new PiecewiseLinearFunction( 2, control_time, control_acts);
			controller->prescribeControlForActuator( i, ccf);
		}
		else 
		{
			Constant* zccf = new Constant(0);
			controller->prescribeControlForActuator( i, zccf);
		}
	}
	model.addController( controller);
}

void setATT(Model& model, SimTK::State &si, double angle_degrees)
{
	const CoordinateSet &knee_r_cs = model.getJointSet().get("knee_r").getCoordinateSet();
    if (angle_degrees == -90)
	{
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.0275 + 0.0065);
		
	}
	else if (angle_degrees == -60)
	{
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.02092232 + 0.0075);
	}
	else if (angle_degrees == -40)
	{
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.012679 + 0.0065);
	}
	else if (angle_degrees == -20)
	{
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.00522225 + 0.005);
	}
	else if (angle_degrees == -15)
	{
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.004 + 0.0045);
	}
	else if (angle_degrees == 0)
	{
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, knee_r_cs.get("knee_anterior_posterior_r").getDefaultValue() + 0.003);
	}

	knee_r_cs.get("knee_anterior_posterior_r").setLocked(si, true);
	knee_r_cs.get("knee_inferior_superior_r").setLocked(si, true);
	knee_r_cs.get("knee_medial_lateral_r").setLocked(si, true);
	knee_r_cs.get("knee_rotation_r").setLocked(si, true);
}

void setKneeAngle(Model& model, SimTK::State &si, double angle_degrees, bool lock_knee_angle, bool constrain_adduction)
{
	const CoordinateSet &knee_r_cs = model.getJointSet().get("knee_r").getCoordinateSet();
	if (angle_degrees == -120)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -2.09439510);  // -120 degrees
		
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.19163894);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.02110966);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.02843407);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.41174209);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00329063);
	}
	else if (angle_degrees == -100)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -1.74533);  // -100 degrees
		
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.23053779);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.00044497);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.0293309);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.40140432);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00504724);
	}
	else if (angle_degrees == -90)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -1.57079);	// -90 degrees

		knee_r_cs.get("knee_adduction_r").setValue(si, -0.24);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.008);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.0275);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.396);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.005);

		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.05235);   // ant load at -90 degrees flexion (+10 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.2574532);   // ant load at -90 degrees flexion (-1 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.1353);   // ant load at -90 degrees flexion (+6 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.1706);   // ant load at -90 degrees flexion (+4 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.1004);   // ant load at -90 degrees flexion (+8 degrees)
	}
	else if (angle_degrees == -80)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -1.39626);  // -80 degrees
			
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.24427703);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.01682137);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.02661332);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.39351699 );
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00483042);

		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.06981);   // ant load at -80 degrees flexion (+10 degrees)
	}
	else if (angle_degrees == -60)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -1.0472);  // -60 degrees
			
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.29941123);
		knee_r_cs.get("knee_rotation_r").setValue(si, -0.00183259);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.02092232);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.38597298);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00403978);

		if (constrain_adduction)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.264511);   // ant load at -60 degrees flexion (+2 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.0648476);   // ant load at -60 degrees flexion (-2 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.157079);   // ant load at -60 degrees flexion (+8 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.191986);   // ant load at -60 degrees flexion (+6 degrees)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.226892);   // ant load at -60 degrees flexion (+4 degrees)

	}
	else if (angle_degrees == -40)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -0.698132);  // -40 degrees
			
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.25397256);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.03301188);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.012679);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.38227168);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00403308);

		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.16667);   // ant load at -40 degrees flexion (+5 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.27142576);   // ant load at -40 degrees flexion (-1 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.148352);   // ant load at -40 degrees flexion (+6 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.13179956);   // ant load at -40 degrees flexion (+7 degrees)
	}
	else if (angle_degrees == -30)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -0.5235987);  // -30 degrees
		
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.27474878);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.01870684);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.008950625);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.38234884);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00444654);

		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.13512278);   // ant load at -30 degrees flexion (+8 degrees)

	}
	else if (angle_degrees == -20)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -0.349066);  // -20 degrees
		
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.295525);
		knee_r_cs.get("knee_rotation_r").setValue(si, 0.0044018);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.00522225);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.382426);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00486);

		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.15708);   // ant load at -20 degrees flexion (+8 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.330431);   // ant load at -20 degrees flexion (-2 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.225715);   // ant load at -20 degrees flexion (+4 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.20943);   // ant load at -20 degrees flexion (+5 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.243225);   // ant load at -20 degrees flexion (+3 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.17453);   // ant load at -20 degrees flexion (+7 degrees)

	}
	else if (angle_degrees == -15)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, -0.26179938);  // -15 degrees
 		
		knee_r_cs.get("knee_adduction_r").setValue(si, -0.279252);
		knee_r_cs.get("knee_rotation_r").setValue(si, -0.03060429);
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, 0.004);
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.384);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, -0.00391863);
		
		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.191992);   // ant load at -15 degrees flexion (+5 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.314152);   // ant load at -15 degrees flexion (-2 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.226952);   // ant load at -15 degrees flexion (+3 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.209442);   // ant load at -15 degrees flexion (+4 degrees)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.157152);   // ant load at -15 degrees flexion (+7 degrees)

	}
	else if (angle_degrees == 0)
	{
		knee_r_cs.get("knee_angle_r").setValue(si, knee_r_cs.get("knee_angle_r").getDefaultValue());
		//knee_r_cs.get("knee_angle_r").setValue(si, 0);
		
		knee_r_cs.get("knee_adduction_r").setValue(si, knee_r_cs.get("knee_adduction_r").getDefaultValue());
		knee_r_cs.get("knee_rotation_r").setValue(si, knee_r_cs.get("knee_rotation_r").getDefaultValue());
		knee_r_cs.get("knee_anterior_posterior_r").setValue(si, knee_r_cs.get("knee_anterior_posterior_r").getDefaultValue());
		knee_r_cs.get("knee_inferior_superior_r").setValue(si, knee_r_cs.get("knee_inferior_superior_r").getDefaultValue());
		//knee_r_cs.get("knee_inferior_superior_r").setValue(si, -0.385578);
		knee_r_cs.get("knee_medial_lateral_r").setValue(si, knee_r_cs.get("knee_medial_lateral_r").getDefaultValue());

		if (constrain_adduction)
			knee_r_cs.get("knee_adduction_r").setValue(si, -0.29408);   // ant load at 0 degrees flexion (0)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.259174);   // ant load at 0 degrees flexion (+2)
			//knee_r_cs.get("knee_adduction_r").setValue(si, -0.31153);   // ant load at 0 degrees flexion (-1)
	}

	knee_r_cs.get("knee_angle_r").setLocked(si, lock_knee_angle);

	knee_r_cs.get("knee_adduction_r").setLocked(si, constrain_adduction);
	//knee_r_cs.get("knee_rotation_r").setLocked(si, true);
	//knee_r_cs.get("knee_anterior_posterior_r").setLocked(si, true);
	//knee_r_cs.get("knee_inferior_superior_r").setLocked(si, true);
	//knee_r_cs.get("knee_medial_lateral_r").setLocked(si, true);
}

void setHipAngle(Model& model, SimTK::State &si, double angle_degrees)
{
	const CoordinateSet &hip_r_cs = model.getJointSet().get("hip_r").getCoordinateSet();

	hip_r_cs.get("hip_flexion_r").setLocked(si, false);
	hip_r_cs.get("hip_flexion_r").setValue(si, angle_degrees * 3.1459 / 180);  
	hip_r_cs.get("hip_flexion_r").setLocked(si, true);
}