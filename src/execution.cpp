#include <iostream>
#include "Utilities/utilities.h"
#include "simulation.hpp"
using namespace std;


int main()
{
	//Set Simulation parameters
	// 	   step size
	// 	   simulation time
	// 	   initiliaze time
	// 	   create vehicles
	//

	double step_size, sim_time, time;
	int N;

	step_size = 0.01;
	sim_time = 20;
	time = 0;
	N = sim_time/step_size;

	Satellite s1;
	Aircraft a1;

	cout << N << endl;
	//Set Envrionment parameters
	// 	   earth shape
	//

	a1.set_earthmodel('r');

	//Link external files to set vehicle parameters

	Matrixop I(3, 3), dI(3, 3);

	I = eye(3);
	I.assign_val(1, 1, 5);
	I.assign_val(2, 2, 3);
	I.assign_val(3, 3, 1);
	
	a1.def_massproperties(I,dI);

	//Initialize variables for dynamics equations
	// 

	Matrixop wo(3, 1), Mo(3, 1), qo(4, 1), euler(3, 1);
	double latitude, longitude, altitude;

	Mo.assign_val(2,1,0);

	wo.assign_val(1, 1, 0);
	wo.assign_val(2, 1, 5);
	wo.assign_val(3, 1, 0);

	euler.assign_val(1, 1, 0);
	euler.assign_val(2, 1, 0);
	euler.assign_val(3, 1, 0);

	a1.init_attitude(euler);
	a1.init_omegab(wo);

	cout << wo.norm() << endl;

	//Start loop of equations of motion
	for (int i = 0; i < N; i++) {

		a1.omega_calc(Mo,step_size);

		time = time + step_size;

	}
	
	cout << time << endl;
	a1.printwb();

	//Record data for post-processing

}