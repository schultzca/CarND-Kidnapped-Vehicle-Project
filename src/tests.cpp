#include <iostream>
#include <random>
#include "particle_filter.h"

using namespace std;

void ParticleFilter_init__create_particles_no_noise() {

	cout << "Test ParticleFilter_init__create_particles_no_noise" << endl;

	ParticleFilter pf;

	double x = 1;
	double y = 1;
	double theta = 1;
	double sig [3] = {0.0, 0.0, 0.0};

	pf.init(x, y, theta, sig);

	bool pass = true;
	if (pf.particles.size() != 100) {
		cout << "Incorrect particle count, " << pf.particles.size() << endl;
		pass = false;
	}
	if (pf.particles[0].x != x) 
	{
		cout << "x coordinate does not match, " << x << "!=" << pf.particles[0].x << endl;
		pass = false;
	}
	if (pf.particles[0].y != y)
	{
		cout << "y coordinate does not match, " << y << "!=" << pf.particles[0].y << endl;
		pass = false;
	}
	if (pf.particles[0].theta != theta)
	{
		cout << "theta coordinate does not match, " << theta << "!=" << pf.particles[0].theta << endl;
		pass = false;
	}

	if (pass) {
		cout << "\t Test Passed!" << endl;
	}
	else {
		cout << "\t Test Failed!" << endl;
	}
}

int main() {
	ParticleFilter_init__create_particles_no_noise();
	return 0;
}