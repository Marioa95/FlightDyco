#pragma once
#include <iostream>
#include "Math_library.hpp"


//Kinematics functions
// 
// addEuler(q1,q2) add two euler angle vectors
// addGibbs(q1,q2) add two classical Rodrigues parameters
// addMRP(q1,q2) add two modified rodrigues parameters
// addPRV(q1,q2) add two principal rotation vector
// DCM2Q(DCM)
// DCM2Gibbs(DCM)
// DCM2MRP(DCM)
// DCM2PRV(DCM)
// Derivatives of attitude representations
// dEuler(q,w)
// dGibbs(q,w)
// dMRP(q,w)
// dPRV(q,omega)
// All attitude representations to DCM
// Euler2DCM(q)
// Gibbs2DCM(q)
// MRP2DCM(q)
// PRV2DCM(q)
// Quaternions to other attitude representations
// Q2Gibbs(q)
// Q2MRP(q)
// Q2PRV(q)
// Euler to other attitude representations
// Euler2Gibbs(q)
// Euler2MRP(q)
// Euler2PRV(q)
// Gibbs to other attitude representations
// 
// MRP to other attitude representations
// 
//

//Quaternions [q0,q1,q2,q3] with scalar term being q0

Matrixop addQ(Matrixop q1, Matrixop q2);
double qnorm(Matrixop q);
Matrixop qinv(Matrixop q);
Matrixop dqdt(Matrixop q,Matrixop w);
Matrixop euler3212q(Matrixop euler);
Matrixop normalize(Matrixop q);

//Direction Cosine Matrix operations

Matrixop q2dcm(Matrixop q);

//Euler Angles

Matrixop euler1(double theta);
Matrixop euler2(double theta);
Matrixop euler3(double theta);
Matrixop dcm2euler(Matrixop dcm);
Matrixop q2euler(Matrixop q);