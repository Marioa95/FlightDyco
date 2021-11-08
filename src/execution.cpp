#include <iostream>
#include "Utilities/utilities.h"
#include "simulation.hpp"
using namespace std;


int main()
{

	Matrixop I(3, 3), dI(3, 3), wo(3, 1), x(7, 1), Mo(3, 1), qo(4, 1), euler(3, 1);
	Satellite s1;
	Aircraft a1;

	double h,tf;
	h = 0.001;
	tf = 20;

	Mo.assign_val(2,1,0);

	I = eye(3);
	I.assign_val(1, 1, 5);
	I.assign_val(2, 2, 3);
	I.assign_val(3, 3, 1);
	
	s1.def_massproperties(I,dI);

	//wo.random();
	wo.assign_val(1, 1, 0);
	wo.assign_val(2, 1, 5);
	wo.assign_val(3, 1, 0);

	euler.assign_val(1, 1, 0);
	euler.assign_val(2, 1, 0);
	euler.assign_val(3, 1, 0);

	qo = euler3212q(euler);

	cout << wo.norm() << endl;

	s1.omega_calc(wo,qo,Mo,h,tf);
	
	
}