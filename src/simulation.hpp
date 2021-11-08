#pragma once
#include "Utilities/utilities.h"

class Simulation {

	double sim_time;
	double step_size;
	int nvehicles;


public:


};

class Environment {
public:
	
};

class Vehicle :public Simulation {


protected:
	
	double* state;
	Matrixop I;
	Matrixop dI;
	Matrixop M;

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

	void forces() {};
	void moments();
	Matrixop dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI);
	void omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo, double h, double tf);

};