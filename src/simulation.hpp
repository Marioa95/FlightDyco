#pragma once
#include "Utilities/utilities.h"

class Simulation {

	double sim_time;
	double step_size;
	int nvehicles;

	//Define flat earth or round earth with a member e.g.
	// String Earth
	// 	   Earth = "Flat"
	// 	Environment class:
	// 	   if Earth = "Flat"
	// 	   then g = constant
	// 	   else if Earth = "Round"
	// 	   then g different and add coriolis term in Dynamics
	//
	// 	   Build state vector for each vehicle as
	// 	   state[]
	//



public:

	double get_simtime() { return sim_time; }
	double get_stepsize() { return step_size; }
	double set_simtime(double t) { sim_time = t; }
	double set_step_size(double h) { step_size = h; }

};

class Environment :public Simulation {
public:
	
};

class Vehicle :public Environment {


protected:
	
	double* state;
	Matrixop I;
	Matrixop dI;
	Matrixop M,F;

public:


	virtual void moments() = 0;
	virtual void forces() = 0;
	void def_massproperties(Matrixop Inertia,Matrixop DInertia);
	
};

class Satellite : public Vehicle {

public:

	//Constructors

	//Functions

	void forces() {};
	void moments();
	Matrixop dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI);
	void omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo, double h, double tf);

};

class Aircraft : public Vehicle {

public:

	//Constructors

	//Functions
	//TRANSLATIONAL EQUATIONS OF MOTION
	void forces();

	//ROTATIONAL EQUATIONS OF MOTION
	
	void moments();
	Matrixop dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI);
	void omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo);

	//FRAMES AND COORDINATE SYSTEMS

	void eci2ecef(Matrixop Aeci,double mu);

};