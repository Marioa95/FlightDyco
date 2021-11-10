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
	
	int N;
	N = get_simtime() / get_stepsize();
	Matrixop w(3, 1), dw(3, 1), q(4, 1), dq(4, 1), euler(3, 1);

	w = wo;
	q = qo;
	M = Mo;
	cout << N << endl;

	for (int i = 1; i < N; i++) {
		dw = dwdt(I, M, w, dI);
		w = eulint(dw, w, get_stepsize());
		moments();
		dq = dqdt(q, w);
		q = eulint(dq, q, get_stepsize());
		q = normalize(q);
		euler = q2euler(q) * (180 / pi);

		omega_b = w;
		attitude_eul = euler;
		attitude_q = q;
	
	}

}

//TRANSLATIONAL EQUATIONS OF MOTION
void Aircraft::forces() {


}

Matrixop Aircraft::dvdt(double mass, Matrixop F, Matrixop v) {

	//Do switch cases for round and flat earth equations
	return F*(1/mass) - cross(omega_b + omega_e(),v);

}
void Aircraft::vbody(Matrixop vo, Matrixop po, Matrixop Fo) {

	int N;
	N = get_simtime() / get_stepsize();
	Matrixop v(3, 1), dv(3, 1), p(4, 1), dp(4, 1);

	v = vo;
	p = po;
	F = Fo;

	for (int i = 1; i < N; i++) {
		dv = dvdt(mass, F, v);
		v = eulint(dv, v, get_stepsize());
		forces();
	}
}

//FRAMES AND COORDINATE SYSTEMS
// ECI (Earth-centered inertial) Nonrotating inertial frame fixed at Earth c.m.
// ECEF(Earth-centered, Earth fixed) Rotating frame defined by rigid Earth at c.m.
// Fv Frame translating with vehicle's c.m.
// Fb Body frame defined by rigid vehicle
//

