#include "simulation.hpp"

Matrixop Environment::omega_e() {
	Matrixop Inertial(3, 1);

	Inertial.assign_val(3,1,1);

	return Inertial * omega_earth;
}

//GRAVITY
Matrixop Environment::gravity(Matrixop position) {
	Matrixop G(3, 1);
	double J2 = 1.0826267e-3;
	double a = 6378137;


}

//ATMOSPHERE