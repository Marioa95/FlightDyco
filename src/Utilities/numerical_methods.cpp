#include <iostream>
#include "Math_library.hpp"
#include <math.h>
using namespace std;

//Euler's Explicit Method

double eulerint(double (*f)(double,double),double t0, double y0, double h, double tf) {

	double* y;
	double *t;
	int N;

	N = tf / h;

	y = new double[N];
	t = new double[N];

	y[0] = y0;
	t[0] = t0;

	for (int i = 1; i < N; i++) {


		t[i] = t[i - 1] + h;
		y[i] = y[i - 1] + h * f(y[i - 1], t[i - 1]);
			
	}

	return y[N-1];

	delete[] y;
	delete[] t;
}

//Matrix integration using euler's method
Matrixop eulint(Matrixop dydt, Matrixop y0, double h) {

	Matrixop y(y0.get_row(),1);

	for (int i = 0; i < y0.get_row(); i++)
		y.assign_val(i + 1, 1, y0.get_ele(i) + dydt.get_ele(i) * h);
	
	return y;
}

double euler(double y0, double h, double dydt) {

	return y0 + h * dydt;
	
}

double f(double y, double t) {
	return 2 - exp(-4 * t) - 2 * y;
}



//Runge kutta 4

//Newton-Raphson Method