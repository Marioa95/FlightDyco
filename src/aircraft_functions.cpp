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
void Aircraft::omega_calc(Matrixop Mo, double h) {
	
	Matrixop w(3, 1), dw(3, 1), q(4, 1), dq(4, 1), euler(3, 1);

	w = omega_b;
	q = euler3212q(attitude_eul);
	M = Mo;

	dw = dwdt(I, M, w, dI);
	w = eulint(dw, w, h);
	moments();
	dq = dqdt(q, w);
	q = eulint(dq, q, h);
	q = normalize(q);
	euler = q2euler(q) * (180 / pi);

	omega_b = w;
	attitude_eul = euler;
	attitude_q = q;
	
}

//TRANSLATIONAL EQUATIONS OF MOTION
void Aircraft::forces() {


}

//Calculates body acceleration wrt earth in body reference frame
Matrixop Aircraft::dvdt(double mass, Matrixop F, Matrixop v) {

	//Do switch cases for round and flat earth equations
	if (get_earthmodel() == 'R' || get_earthmodel() == 'r')

		//Velocity of body wrt Earth in geodetic coordinates. F in body frame
		return (G2B(attitude_eul) * F) * (1 / mass) + G2D(data[13]) * gravity(position, data[13]) - (E2G(data[13], data[14]) * omega_e().skewsym() * E2G(data[13], data[14]).trans() * v) * 2 - E2G(data[13], data[14]) * omega_e().skewsym() * omega_e().skewsym() * position;

	else if (get_earthmodel() == 'F' || get_earthmodel() == 'f')

		return F * (1 / mass) + gravity(position);



}
void Aircraft::vbody(Matrixop Fo, double h) {

	Matrixop v(3, 1), dv(3, 1), p(3, 1), dp(3, 1);

	v = velocity;
	p = position;
	F = Fo;

	if (get_earthmodel() == 'R' || get_earthmodel() == 'r') {
	
		dv = dvdt(mass, F, v);
		v = eulint(dv, v, h);
		forces();

		

	}

	else if (get_earthmodel() == 'F' || get_earthmodel() == 'f') {
		
		dv = dvdt(mass, F, v);
		v = eulint(dv, v, h);
		forces();

	}

		
}



