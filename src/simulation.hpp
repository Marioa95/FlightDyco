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
	void set_simtime(double t) { sim_time = t; }
	void set_step_size(double h) { step_size = h; }

};

class Environment :public Simulation {
	//Earth parameters defined by WGS-84
	#define omega_earth 7292115e-11 // rad/s
	#define Rearth //Mean radius of the Earth
	#define Mu 3986005e8 //Earth's gravitational constant


public:

	Matrixop omega_e();
	//GRAVITY


	//ATMOSPHERE

};

class Vehicle :public Environment {


protected:
	
	double* data;
	Matrixop omega_b, position, attitude_eul,attitude_q;
	Matrixop I;
	double mass;
	Matrixop dI,dm;
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

	//DATA STORAGE


	//ROTATIONAL EQUATIONS OF MOTION
	
	void moments();
	Matrixop dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI);
	void omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo);

	//FRAMES AND COORDINATE SYSTEMS

	void eci2ecef(Matrixop Aeci,double mu);

	//TRANSLATIONAL EQUATIONS OF MOTION
	
	void forces();
	Matrixop dvdt(double mass,Matrixop F, Matrixop v);
	void vbody(Matrixop vo,Matrixop po,Matrixop Fo);
};