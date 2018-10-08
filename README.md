# Knee Simulation

Modeling and simulation of human knee joint, using rigid and deformable bodies for ligaments, meniscii and articular cartilage.
Objective is to estimate the mechanical behavior of knee structures during the execution of some tasks.

• Modeling of knee structures

• Simulation of some tasks

• Estimation of joint parameters


It was implemented as a Diploma Thesis in Electrical and Computer Engineering Department in University of Patras [VVR Lab](http://www.vvr.ece.upatras.gr/index.php/el/studies/theses/completed/133-human-knee-simulation-based-on-realistic-musculoskeletal-models). 

Video presentation: [youtube link](https://www.youtube.com/watch?v=55KIqKJ46kc "here"). 

![<Here is a Presentation .gif>](https://j.gifs.com/RogZGE.gif "This is a Presentation .gif")

## Requirements
*	OpenSim 3.2
*	Simbody 
	
## Build

First build the plugins contained in the repository (CustomAnalysisPlugin & CustomLigamentPlugin) and link the headers and extracted .dll's to the solution contained in the KneeSimulation/ACLproj folder.
Also link the OpenSim and Simbody libraries.

### Notes
Tested with Visual Studio 2010, 2015 in Windows 7.
