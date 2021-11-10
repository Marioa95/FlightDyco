#include "simulation.hpp"

//Defines the mass properties of the vehicle such as inertia, mass and inertia rate
void Vehicle::def_massproperties(Matrixop Inertia, Matrixop dInertia) {

	I = Inertia;
	dI = dInertia;
}

//FRAMES AND COORDINATE SYSTEMS
// ECI (Earth-centered inertial) Nonrotating inertial frame fixed at Earth c.m.
// ECEF(Earth-centered, Earth fixed) Rotating frame defined by rigid Earth at c.m.
// Fv Frame translating with vehicle's c.m.
// Fb Body frame defined by rigid vehicle
//

Matrixop Vehicle::eci2ecef(double mu) {

	return euler3(mu);
}

Matrixop ecef2b(double yaw, double pitch, double roll) {
	
	return euler321(yaw, pitch, roll);

}

Matrixop Vehicle::ecef2NED(double lat, double lon) {

	Matrixop middle(3, 3);

	middle.assign_val(1, 3, 1);
	middle.assign_val(2, 2, 1);
	middle.assign_val(3, 1, -1);

	return euler3(lon) * middle * (euler3(lat).trans());
}