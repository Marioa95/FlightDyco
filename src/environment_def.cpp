#include "simulation.hpp"

//EARTH GEOMETRY
double a = 6378137; //semi-major axis
double e = 0.08181919; //eccentricity
double b = 6356752; //semi-minor axis
double f = (a - b) / a; //flattening parameter

Matrixop Environment::omega_e() {
	Matrixop Inertial(3, 1);

	Inertial.assign_val(3,1,1);

	return Inertial * omega_earth;
}
//Deflection angle that relates geocentric coordinates with geodetic coordinates
double Environment::delta(double altitude,double latd) {
	latd = latd * deg2rad;
	return f * sin(2 * latd) * (1 - f * 0.5 - altitude / Ro(latd));
}

double Environment::Ro(double latd) {
	latd = latd * deg2rad;
	return a * (1 - f * 0.5 * (1 - cos(2 * latd)) + (5 / 16) * pow(f, 2) * (1 - cos(4 * latd)));
}

//GRAVITY

//Round earth gravitational acceleration
Matrixop Environment::gravity(Matrixop position, double latd) {
	Matrixop g(3, 1);

	//Elliptical earth gravitational acceleration
	double latc,a,b;
	double C = -4.841668e-4;
	latc = latd - delta(-position.get_ele(3), latd);
	a = Mu / pow(position.norm(), 2);
	b = 3 * sqrt(5) * C * pow(a / position.norm(), 2);

	g.assign_val(1, 1, a * -b * sin(latc) * cos(latc));
	g.assign_val(3, 1, a * (1 + 0.5 * b * (3 * pow(sin(latc), 2) - 1)));
	

	return g;
}
Matrixop Environment::gravity(Matrixop position) {
	Matrixop g(3, 1);

	//Flat earth gravity equation
	g.assign_val(3, 1, Mu / pow(Rearth + position.get_ele(3), 2));

	return g;
}

//ATMOSPHERE

