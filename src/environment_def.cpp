#include "simulation.hpp"

Matrixop Environment::omega_e() {
	Matrixop Inertial(3, 1);

	Inertial.assign_val(3,1,1);

	return Inertial * omega_earth;
}

//GRAVITY


//ATMOSPHERE