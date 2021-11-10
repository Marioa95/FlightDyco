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