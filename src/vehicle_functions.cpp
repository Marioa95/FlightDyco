#include "simulation.hpp"

void Vehicle::storedata() {

	data[0] = get_simtime() * get_stepsize();
	data[1] = position.get_ele(1);
	data[2] = position.get_ele(2);
	data[3] = position.get_ele(3);
	data[7] = attitude_eul.get_ele(1);
	data[8] = attitude_eul.get_ele(2);
	data[9] = attitude_eul.get_ele(3);
	data[10] = omega_b.get_ele(1);
	data[11] = omega_b.get_ele(2);
	data[12] = omega_b.get_ele(3);


}

//Defines the mass properties of the vehicle such as inertia, mass and inertia rate
void Vehicle::def_massproperties(Matrixop Inertia, Matrixop dInertia) {

	I = Inertia;
	dI = dInertia;
}

//FRAMES AND COORDINATE SYSTEMS
// I - ECI (Earth-centered inertial) Nonrotating inertial frame fixed at Earth c.m.
// E - ECEF(Earth-centered, Earth fixed) Rotating frame defined by rigid Earth at c.m.
// V - Frame translating with vehicle's c.m.
// B - Body frame defined by rigid vehicle
// G - Geographic frame
// W - Wind reference frame
//

Matrixop Vehicle::I2E(double mu) {

	return euler3(mu);
}

Matrixop G2B(double yaw, double pitch, double roll) {
	
	return euler321(yaw, pitch, roll);

}

Matrixop Vehicle::E2G(double lat, double lon) {

	return euler2(180) * euler2(90 - lat) * euler3(lon);
}

Matrixop Vehicle::B2W(double alpha, double beta) {

	return euler2(alpha) * euler3(beta);
}

Matrixop Vehicle::G2V(double fpa, double heading) {

	return euler2(fpa) * euler3(heading);
}

double Vehicle::get_alpha(Matrixop v_B) {

	double alpha;

	alpha = atan(v_B.get_ele(3) / v_B.get_ele(1));

	return alpha;
}

double Vehicle::get_beta(Matrixop v_B) {

	double beta;

	beta = asin(v_B.get_ele(2) / v_B.norm());

	return beta;

}

double Vehicle::get_fpa(Matrixop v_G) {

	double fpa,n;

	n = sqrt(pow(v_G.get_ele(1), 2) + pow(v_G.get_ele(2), 2));

	fpa = atan(-v_G.get_ele(3) / n);

	return fpa;

}

double Vehicle::get_heading(Matrixop v_G) {

	double heading;

	heading = atan(v_G.get_ele(2) / v_G.get_ele(1));

	return heading;

}