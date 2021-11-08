#pragma once
#include "Utilities/utilities.h"

class Simulation {

	double time;
	double step_size;
	int nvehicles;


public:


};

class Environment:public Simulation {
public:
	
};

class Vehicle :public Environment {


protected:
	
	double* state;
	Matrixop I;
	Matrixop dI;
	Matrixop M;

public:


	virtual void moments() = 0;
	virtual void forces() = 0;
	virtual void def_massproperties(Matrixop Inertia,Matrixop DInertia) = 0;
	
};

class Satellite : public Vehicle {

public:

	//Constructors

	//Functions

	void forces() {};
	void moments();
	void def_massproperties(Matrixop Inertia, Matrixop dInertia);
	Matrixop dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI);
	void omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo, double h, double tf);

};

class Aircraft : public Vehicle {

public:

	//Constructors

	//Functions

	void forces() {};
	void moments();
	void def_massproperties(Matrixop Inertia, Matrixop dInertia);
	Matrixop dwdt(Matrixop I, Matrixop M, Matrixop w, Matrixop dI);
	void omega_calc(Matrixop wo, Matrixop qo, Matrixop Mo, double h, double tf);

};