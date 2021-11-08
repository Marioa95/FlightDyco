#include "simulation.hpp"

//Defines the mass properties of the vehicle such as inertia, mass and inertia rate
void Vehicle::def_massproperties(Matrixop Inertia, Matrixop dInertia) {

	I = Inertia;
	dI = dInertia;
}