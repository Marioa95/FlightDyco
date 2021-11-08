#include "simulation.hpp"

//This file contains the functions to calculate the dynamics of a Aircraft
// forces()
// moments()
// 
//

//ROTATIONAL EQUATIONS OF MOTION
void Aircraft::moments() {

	Matrixop Moments(3, 1);

	Moments.assign_val(2, 1, 10);
	M = Moments;


}

//Calculates the angular acceleration
Matrixop Aircraft::dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI) {

	return I.inv() * (M - dI * w - cross(w, I * w));

}

//Calculates the angular velocity in body frame by integrating angular acceleration
void Aircraft::omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo) {
	double h, tf;

	h = get_stepsize();
	tf = get_simtime();

	int N;
	N = tf / h;
	Matrixop w(3, 1), dw(3, 1), q(4, 1), dq(4, 1), x(N, 3), euler(3, 1);

	w = wo;
	q = qo;
	M = Mo;
	cout << N << endl;

	for (int i = 1; i < N; i++) {
		dw = dwdt(I, M, w, dI);
		w = eulint(dw, w, h);
		moments();
		dq = dqdt(q, w);
		q = eulint(dq, q, h);
		q = normalize(q);
		euler = q2euler(q) * (180 / pi);
		//euler.display();


		//x.assign_val(i, 1, w.get_ele(0));
		//x.assign_val(i, 2, w.get_ele(1));
		//x.assign_val(i, 3, w.get_ele(2));
		/*x.assign_val(i, 1, q.get_ele(0));
		x.assign_val(i, 2, q.get_ele(1));
		x.assign_val(i, 3, q.get_ele(2));
		x.assign_val(i, 4, q.get_ele(3));*/
		x.assign_val(i, 1, euler.get_ele(0));
		x.assign_val(i, 2, euler.get_ele(1));
		x.assign_val(i, 3, euler.get_ele(2));

	}

	writefile(x);
	cout << w.norm();
}

//TRANSLATIONAL EQUATIONS OF MOTION
void Aircraft::forces() {


}


//FRAMES AND COORDINATE SYSTEMS
// ECI (Earth-centered inertial) Nonrotating inertial frame fixed at Earth c.m.
// ECEF(Earth-centered, Earth fixed) Rotating frame defined by rigid Earth at c.m.
// Fv Frame translating with vehicle's c.m.
// Fb Body frame defined by rigid vehicle
//

void Aircraft::eci2ecef(Matrixop Aeci,double mu) {
	
	euler3(mu);

}