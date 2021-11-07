#include <iostream>
#include "Math_library.hpp"
#include <math.h>
#include <fstream>
using namespace std;

//////////////////////////////////////////////
//Constructors
//////////////////////////////////////////////
Matrixop::Matrixop() {}
Matrixop::Matrixop(int row=1, int col=1) {
	nrow = row;
	ncol = col;
	nelements = row * col;

	try { array = new double[nelements]; }
	catch (bad_alloc xa) { cerr << "Error: Matrix memory allocation failed\n"; exit(1); }

	for (int i = 0; i < nelements; i++) {
		*(array + i) = 0;
	}
}

Matrixop::Matrixop(const Matrixop& mat) {

	nrow = mat.nrow;
	ncol = mat.ncol;
	nelements = mat.nelements;

	try { array = new double[nelements]; }
	catch (bad_alloc xa) { cerr << "Error: Matrix memory allocation failed\n"; exit(1); }


	for (int i = 0; i < nelements; i++) {
		*(array + i) = *(mat.array + i);
	}
	//cout << "Copying" <<endl;
}

//////////////////////////////////////////////
//Functions
//////////////////////////////////////////////

//Return the skew symmetric matrix of the object that uses the function
Matrixop Matrixop::skewsym() {
	if (nrow!=3 && ncol != 1) { cerr << "Error: this is not a 3x1 vector for skewsym()\n"; }
	
	Matrixop temp(3, 3);
	*(temp.array+1) = -*(array+2);
	*(temp.array+2) = *(array+1);
	*(temp.array+3) = *(array+2);
	*(temp.array+5) = -*(array);
	*(temp.array+6) = -*(array+1);
	*(temp.array+7) = *(array);

	return temp;
}

void Matrixop::display() {
	double* temp = array;
	for (int i = 0; i < nrow; i++) {
		for (int j = 0; j < ncol; j++){
			cout << *(array + ncol*i+ j) << "\t";
		}
		cout << endl;
	}
	cout << "\n\n";
	array = temp;
}

//Return the transpose matrix of the object that uses the function
Matrixop Matrixop::trans() {
	Matrixop trans(ncol, nrow);

	for (int i = 0; i < ncol; i++) {
		for (int j = 0; j < nrow; j++) {
			*(trans.array + ncol * i+j) = *(array + ncol * j + i);
		}
	}
	return trans;
}

Matrixop Matrixop::adj() {

	Matrixop c(nrow, ncol),temp(nrow, ncol);

	for (int k = 0; k < nelements; k++)
		*(temp.array + k) = *(array + k);

	for (int i = 0; i < nrow; i++) {
		for (int j = 0; j < ncol; j++) {
			*(c.array + ncol*i+j) = pow(-1.0, i+j)*(temp.submat(i+1, j+1)).det2();
		}
	}
	return c.trans();
}
//Return the inverse matrix of the object that uses the function
Matrixop Matrixop::inv(){
	Matrixop inverse(nrow,ncol),c(nrow,ncol);
	
	if (nrow != ncol)
	{
		cerr << "Error: inv() only works with square matrices "; exit(1);
	}

	for (int i = 0; i < nelements; i++)
		*(c.array + i) = *(array + i);

	inverse = c.adj();
	inverse = inverse * (1 / c.det3());


	return inverse;
}
void Matrixop::assign_val(int row,int col, double value) {
	
	if(row > nrow || col > ncol)
	{cerr << "Error: number of elements was exceeded "; exit(1);}

	*(array + (row-1)*ncol + (col-1)) = value;
}

void Matrixop::random() {

	for (int i=0; i < nelements;i++)
		*(array + i ) = rand() % 20;


}

//Converts the object to an identity matrix of the same size as the object
void Matrixop::identity() {
	if (nrow != ncol)
	{cerr << "Error: identity() only works with square matrices "; exit(1);}

	for (int i = 0; i < nelements; i++)
		*(array + ncol * i + i) = 1;
}

double Matrixop::get_ele(int nelement) {
	double element;

	//element = *(array + (col - 1) + (row - 1) * ncol);
	element = *(array + nelement);
	return element;
}

double Matrixop::det2() {
	if (nrow = !ncol)
	{cerr << "Error: The matrix is not a square matrix. Determinant cannot be computed "; exit(1);}

	double determinant;

	determinant = *(array) * *(array + 3) - *(array + 2) * *(array + 1);

	return determinant;

}
//Computes the submatrix of an square matrix by eliminating the row and column entered in the argument
Matrixop Matrixop::submat(int row,int col) {

	Matrixop submat(nrow-1,ncol-1);
	int k = 0;

	for (int i = 0; i < nrow; i++) {
		for (int j = 0; j < ncol; j++) {
			if (j == (col - 1) || i == (row-1))
				continue;
			else
				*(submat.array + k) = *(array+ncol*i+j);
			k++;
		}
	}
	return submat;
}
double Matrixop::det3() {

	if (nrow =!ncol)
	{cerr << "Error: The matrix is not a square matrix. Determinant cannot be computed "; exit(1);}
	
	double determinant = 0 ;
	Matrixop c(3,3);

	for (int k = 0; k < nelements; k++)
		*(c.array + k) = *(array + k);


	for (int i = 0; i < ncol; i++)
		determinant = determinant + pow(-1.0,i) * *(c.array + i) * (c.submat(1, i+1)).det2();

	return determinant;
}
double dot(Matrixop vec1, Matrixop vec2) {

	double result = 0;

	for (int i = 0; i < vec1.get_nele(); i++)
		result = result + vec1.get_ele(i) * vec2.get_ele(i);

	return result;
}
Matrixop Matrixop::remove() {
	Matrixop temp(nrow - 1, ncol);

	for (int i = 0; i < nelements-1; i++) {
			temp.array[i] = array[i+1];
	}

	return temp;
}
double Matrixop::norm() {
	double result = 0;

	for (int i = 0; i < nelements; i++)
		result = result + array[i]*array[i];


	return sqrt(result);
}
Matrixop cross(Matrixop vec1, Matrixop vec2) {

	Matrixop result(vec1.get_row(),vec2.get_col());

	result.assign_val(1,1,vec1.get_ele(1) * vec2.get_ele(2) - vec1.get_ele(2) * vec2.get_ele(1));
	result.assign_val(2,1,vec1.get_ele(2) * vec2.get_ele(0) - vec1.get_ele(0) * vec2.get_ele(2));
	result.assign_val(3,1,vec1.get_ele(0) * vec2.get_ele(1) - vec1.get_ele(1) * vec2.get_ele(0));

	return result;
}
//Returns a size-by-size identity matrix 
Matrixop eye(int size) {
	Matrixop I(size, size);

	for (int i = 1; i < size+1; i++)
		I.assign_val(i, i, 1);

	return I;
}

//Write array on a .txt file comma separated
void writefile(Matrixop x) {

	ofstream myfile;
	myfile.open("attitude.txt");

	for (int j = 0; j < x.get_row(); j++) {
		for (int i = 0; i < x.get_col(); i++) {
			myfile << x.get_ele(j * x.get_col() + i);
			if (i != x.get_col() - 1)
				myfile << ',';
			else
			{
				myfile << '\n';
			}
		}
		
	}

	myfile.close();
	
}

//////////////////////////////////////////////
//Operators Overloading
//////////////////////////////////////////////

Matrixop Matrixop::operator +(const Matrixop &mat2) {
	
	if(nrow!=mat2.nrow || ncol!=mat2.ncol)
	{cerr << "Error: matrices have different dimensions 'operator +' "; exit(1);}

	Matrixop sum(nrow,ncol);

	for (int i=0;i<nelements;i++)
	*(sum.array+i)=*(array+i) + (*(mat2.array+i));

	return sum;
}
Matrixop Matrixop::operator -(const Matrixop& mat2) {

	if (nrow != mat2.nrow || ncol != mat2.ncol)
	{
		cerr << "Error: matrices have different dimensions 'operator +' "; exit(1);
	}

	Matrixop dif(nrow, ncol);

	for (int i = 0; i < nelements; i++)
		*(dif.array + i) = *(array + i) - (*(mat2.array + i));

	return dif;
}
Matrixop Matrixop::operator *(const Matrixop& mat2) {

	if (ncol != mat2.nrow)
	{cerr << "Error: Matrices cannot be multiplied "; exit(1);}

	Matrixop mult(nrow,mat2.ncol);
	double sum=0;

	for (int i = 0; i < nrow; i++)
		for (int j = 0; j < mat2.ncol; j++) {
			for (int k = 0; k < ncol; k++) {
				sum = sum + array[ncol * i + k]  * mat2.array[mat2.ncol * k + j];
			}
			mult.array[mat2.ncol * i + j] = sum;
			sum = 0;
		}

	/*
	for (int i = 0; i < nrow; i++)
		for (int j = 0; j < mat2.ncol; j++) {
			for (int k = 0; k < ncol; k++) {
				sum = sum + *(array + ncol * i + k) * (*(mat2.array + mat2.ncol * k + j));
			}
			*(mult.array + mat2.ncol * i+j) = sum;
			sum = 0;
		}
	*/

	return mult;
}
Matrixop Matrixop::operator *(const double scalar) {
	Matrixop mult(nrow, ncol);

	for (int i = 0; i < nrow; i++) {
		for (int j = 0; j < ncol; j++) {
			*(mult.array + ncol * i + j) = *(array + ncol * i + j) * scalar;
		}
	}

	return mult;
}
Matrixop& Matrixop::operator =(const Matrixop &mat2) {

	if (this == &mat2) 
		return *this;

	else
	{
		delete[] array;
		nelements = mat2.nelements;
		nrow = mat2.nrow;
		ncol = mat2.ncol;

		array = new double[nelements];

		for (int i = 0; i < nelements; i++)
				*(array + i) = *(mat2.array + i);

		return *this;
	}

	
}

//////////////////////////////////////////////
//Destructors
//////////////////////////////////////////////
Matrixop::~Matrixop(){

//	cout << "destructing " << endl;
	delete[] array;
}

