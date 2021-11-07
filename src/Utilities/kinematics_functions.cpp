#include <iostream>
#include "kinematics.h"
#include <math.h>
using namespace std;

//QUATERNIONS FUNCTIONS

Matrixop addQ(Matrixop q1, Matrixop q2) {

	Matrixop temp(3, 1), sum(4, 1);
	sum.assign_val(1, 1, q1.get_ele(0) * q2.get_ele(0) - dot(q1.remove(), q2.remove()));
	temp = (q2.remove() * q1.get_ele(0) + q1.remove() * q2.get_ele(0) + cross(q1.remove(), q2.remove()));
	sum.assign_val(2, 1, temp.get_ele(0));
	sum.assign_val(3, 1, temp.get_ele(1));
	sum.assign_val(4, 1, temp.get_ele(2));

	return sum;
	/*
	array[0] = q1.array[0] * q2.array[0] - q1.array[1] * q2.array[1] - q1.array[2] * q2.array[2] - q1.array[3] * q2.array[3];
	array[1] = q1.array[1] * q2.array[0] + q1.array[0] * q2.array[1] + q1.array[3] * q2.array[2] - q1.array[2] * q2.array[3];
	array[2] = q1.array[2] * q2.array[0] - q1.array[3] * q2.array[1] + q1.array[0] * q2.array[2] + q1.array[1] * q2.array[3];
	array[3] = q1.array[3] * q2.array[0] + q1.array[2] * q2.array[1] - q1.array[1] * q2.array[2] + q1.array[0] * q2.array[3];
	*/

}
double qnorm(Matrixop q) {

	double result = 0;

	for (int i = 0; i < q.get_nele(); i++)
		result = result + q.get_ele(i) * q.get_ele(i);

	return result;

}
Matrixop qinv(Matrixop q) {

	Matrixop inv(4, 1);

	inv.assign_val(1, 1, q.get_ele(0) / qnorm(q));
	inv.assign_val(2, 1, -q.get_ele(1) / qnorm(q));
	inv.assign_val(3, 1, -q.get_ele(2) / qnorm(q));
	inv.assign_val(4, 1, -q.get_ele(3) / qnorm(q));

	return inv;
}
Matrixop dqdt(Matrixop q,Matrixop w){
	Matrixop dq(4,1),B(4,3);
	B.assign_val(1, 1, -q.get_ele(1));
	B.assign_val(1, 2, -q.get_ele(2));
	B.assign_val(1, 3, -q.get_ele(3));
	B.assign_val(2, 1, q.get_ele(0));
	B.assign_val(2, 2, -q.get_ele(3));
	B.assign_val(2, 3, q.get_ele(2));
	B.assign_val(3, 1, q.get_ele(3));
	B.assign_val(3, 2, q.get_ele(0));
	B.assign_val(3, 3, -q.get_ele(1));
	B.assign_val(4, 1, -q.get_ele(2));
	B.assign_val(4, 2, q.get_ele(1));
	B.assign_val(4, 3, q.get_ele(0));

	dq = (B * w) * 0.5;

	return dq;

}
Matrixop normalize(Matrixop q) {
	Matrixop normalized(4, 1);

	normalized = q * (1/sqrt(qnorm(q)));
	
	return normalized;

}

//Converts yaw, pitch, roll to quaternions. Input in deg
Matrixop euler3212q(Matrixop euler) {
	Matrixop q(4, 1);
	double c1, s1, c2, s2, c3, s3;
	
	euler.assign_val(1, 1, euler.get_ele(0) * deg2rad);
	euler.assign_val(2, 1, euler.get_ele(1) * deg2rad);
	euler.assign_val(3, 1, euler.get_ele(2) * deg2rad);

	c1 = cos(euler.get_ele(0) / 2);
	s1 = sin(euler.get_ele(0) / 2);
	c2 = cos(euler.get_ele(1) / 2);
	s2 = sin(euler.get_ele(1) / 2);
	c3 = cos(euler.get_ele(2) / 2);
	s3 = sin(euler.get_ele(2) / 2);

	q.assign_val(1, 1, c1 * c2 * c3 + s1 * s2 * s3);
	q.assign_val(2, 1, s1 * c2 * c3 - c1 * s2 * s3);
	q.assign_val(3, 1, c1 * s2 * c3 + s1 * c2 * s3);
	q.assign_val(4, 1, c1 * c2 * s3 - s1 * s2 * c3);

	return q;

}

//DCM FUNCTIONS

Matrixop q2dcm(Matrixop q) {

	Matrixop I(3, 3), DCM(3, 3);
	I = eye(3);

	DCM = q.remove() * 2 * (q.remove()).trans() + I * (q.get_ele(0) * q.get_ele(0) - pow((q.remove()).norm(), 2)) - (q.remove()).skewsym() * 2 * q.get_ele(0);

	return DCM;

}

//EULER ANGLES FUNCTIONS

Matrixop euler1(double theta) {
	Matrixop euler(3, 3);

	euler.assign_val(1, 1, 1);
	euler.assign_val(2, 2, cos(theta));
	euler.assign_val(2, 3, sin(theta));
	euler.assign_val(3, 2, -sin(theta));
	euler.assign_val(3, 3, cos(theta));

	return euler;
}
Matrixop euler2(double theta) {

	Matrixop euler(3, 3);

	euler.assign_val(1, 1, cos(theta));
	euler.assign_val(1, 3, -sin(theta));
	euler.assign_val(2, 2, 1);
	euler.assign_val(3, 1, sin(theta));
	euler.assign_val(3, 3, cos(theta));

	return euler;
}
Matrixop euler3(double theta) {

	Matrixop euler(3, 3);
	euler.assign_val(1, 1, cos(theta));
	euler.assign_val(1, 2, sin(theta));
	euler.assign_val(2, 1, -sin(theta));
	euler.assign_val(2, 2, cos(theta));
	euler.assign_val(3, 3, 1);

	return euler;
}
Matrixop dcm2euler(Matrixop dcm) {
	Matrixop euler(3, 1);

	euler.assign_val(3, 1, atan2(dcm.get_ele(1), dcm.get_ele(0)));
	euler.assign_val(2, 1, -asin(dcm.get_ele(2)));
	euler.assign_val(1, 1, atan2(dcm.get_ele(5), dcm.get_ele(8)));

	return euler;

}
Matrixop q2euler(Matrixop q) {
	Matrixop euler(3, 1), dcm(3, 3);

	dcm = q2dcm(q);
	euler = dcm2euler(dcm);

	return euler;
}

