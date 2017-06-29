#include <iostream>
#include <random>
#include "particle_filter.h"

bool ParticleFilter_init__create_particles_no_noise() {
	ParticleFilter pf;

	double x = 1;
	double y = 1;
	double theta = 1;
	double sig [3] = {0.3, 0.3, 0.3};

	pf.init(x, y, theta, sig);

	bool pass = true;
	if (pf.particles.size() == 100) pass = false;
	if (pf.particles[0].x == x) pass = false;
	if (pf.particles[0].y == y) pass = false;
	if (pf.particles[0].theta == theta) pass = false;

	Particle p = pf.particles[0];
	std::cout << pf.particles.size() << std::endl;
	std::cout << p.x << std::endl;
	std::cout << p.y << std::endl;
	std::cout << p.theta << std::endl;

	return pass;
}

int main() {
	std::cout << "ParticleFilter_init__create_particles_no_noise" << ", " << ParticleFilter_init__create_particles_no_noise() << std::endl;
	return 0;
}