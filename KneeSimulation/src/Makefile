CXX=clang++

CXXFLAGS=-Wall -O2 -g -std=c++11 -Wno-unsequenced

INCPATH=-isystem$(HOME)/Apps/simbody/simbody331/include \
		-isystem$(HOME)/Apps/opensim/opensim32/sdk/include \
		-isystem$(HOME)/Projects/VegaFEM/VegaFEM-v2.1/libraries/include 

LIBRARYPATH=-L$(HOME)/Apps/opensim/opensim32/lib 
			#\ -L$(HOME)/Projects/VegaFEM/VegaFEM-v2.1/libraries/lib 
			#\ -L$(HOME)/Projects/VegaFEM/VegaFEM-v2.1/libraries/glui/glui-2.35/src/lib 

LIBS=-lSimTKcommon -lSimTKmath -lSimTKsimbody \
	 -losimActuators -losimAnalyses -losimCommon -losimLepton \
	 -losimSimulation -losimTools \
	 -lGL -lGLU -lglut \
	 -lstdc++ -lpthread 
	 #-lsceneObject -lintegratorSparse -lintegrator -lelasticForceModel -lforceModel \
	 #-lSimTKcommon_d -lSimTKmath_d -lSimTKsimbody_d \
	 #-losimActuators_d -losimAnalyses_d -losimCommon_d -losimLepton_d \
	 #-losimSimulation_d -losimTools_d \
	 # VEGA FEM
	 # -lsparseMatrix -lloadList -linsertRows -llighting -lconfigFile -lvolumetricMesh \
	 # -lgetopts -lcamera -lgraph -lisotropicHyperelasticFEM -lstvk -lcorotationalLinearFEM \
	 # -lpolarDecomposition -lminivector -lmatrixIO -lmassSpringSystem -lobjMesh -limageIO -lsparseSolver -lmatrix \
	 # $(HOME)/Apps/spooles/LinSol/srcST/Bridge.a $(HOME)/Apps/spooles/LinSol/srcMT/BridgeMT.a \
	 # $(HOME)/Apps/spooles/MT/src/spoolesMT.a $(HOME)/Apps/spooles/spooles.a \
#----------------------------------------------------------------------------------------------------
all: aclsim

aclsim: main.cpp osimutils.cpp addMeniscusToModel.cpp addKneeContactGeometries.cpp ACLsimulatorimpl.cpp
	$(CXX) $(CXXFLAGS) $(INCPATH) $^ -o $@ $(LIBRARYPATH) $(LIBS)

run:
	./aclsim

clean: 
	$(RM) aclsim