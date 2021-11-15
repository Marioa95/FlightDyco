#include "simulation.hpp"

Matrixop Environment::omega_e() {
	Matrixop Inertial(3, 1);

	Inertial.assign_val(3,1,1);

	return Inertial * omega_earth;
}

//GRAVITY
//gravitational model spherical harmonics
Matrixop Environment::gravity(Matrixop position) {
	Matrixop G(3, 1);
	double g;
	double J2 = 1.0826267e-3;
	double a = 6378137;
	double e = 0.08181919;
	
	//Round earth gravity equation

	//Flat earth gravity equation
	g = Mu/pow(Rearth + position.get_ele(3),2);

}

//ATMOSPHERE

