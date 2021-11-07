#pragma once
#include <iostream>

#define pi 3.14159265
#define deg2rad pi/180

using namespace std;

//Matrix operations class
//


class Matrixop
{
	int ncol;
	int nrow;
	int nelements;
	double* array;

public:
	
	//Constructors
	Matrixop();
	Matrixop(int row, int col);
	Matrixop(const Matrixop& mat);

	//Functions
	int get_row() { return nrow; }
	int get_col() { return ncol; }
	int get_nele() { return nelements; }
	double get_ele(int nelement);
	void display();
	void assign_val(int row, int col, double value);
	void random();

	//Matrix properties
	Matrixop skewsym(); 
	Matrixop trans();
	Matrixop adj();
	Matrixop inv();
	void identity();
	double det2();
	Matrixop submat(int row,int col);
	double det3();
	double norm();
	Matrixop remove();

	//Operators Overloading
	Matrixop operator +(const Matrixop &mat2);
	Matrixop operator -(const Matrixop& mat2);
	Matrixop operator *(const Matrixop& mat2);
	Matrixop operator *(const double scalar);
	Matrixop& operator =(const Matrixop &mat2);
	//Destructors
	~Matrixop();

};

Matrixop eye(int size);
double dot(Matrixop vec1, Matrixop vec2);
Matrixop cross(Matrixop vec1, Matrixop vec2);
void writefile(Matrixop x);

//Numerical Methods Functions
double eulerint(double (*f)(double,double),double t0, double y0, double h, double tf);
Matrixop eulint(Matrixop dydt, Matrixop y0, double h);
double euler(double y0, double h, double dydt);
double f(double y, double t);


