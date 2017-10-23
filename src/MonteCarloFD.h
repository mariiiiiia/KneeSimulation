/*
*	Perform Monte Carlo analysis for active knee flexion experiment,
*	repeating this task <iteration> times
*	and changing a variable (meniscus stiffness, ligament stiffness etc) through uniform distribution
*/
void performMCFD_flexion(Model model, int iterations);
/*
*	Perform Monte Carlo analysis for anterior tibial loads experiment,
*	repeating this task <iteration> times
*	and changing a variable (meniscus stiffness, ligament stiffness etc) through uniform distribution
*/
void performMCFD_atl(Model model, int iterations);
