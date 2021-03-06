#include <iostream>
#include <random>
#include <math.h>
#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

bool equal(double a, double b) {
	return fabs(a - b) < 0.0001;
}

void ParticleFilter_init() {

	cout << "Test ParticleFilter_init" << endl;

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

void ParticleFilter_prediction_zero_yaw_rate_x_only() {

	cout << "Test ParticleFilter_prediction_zero_yaw_rate_x_only" << endl;

	ParticleFilter pf;

	double x = 1;
	double y = 1;
	double theta = 0;
	double sig [3] = {0.0, 0.0, 0.0};

	pf.init(x, y, theta, sig);
	
	double dt = 1;
	double velocity = 1;
	double yaw_rate = 0;

	pf.prediction(dt, sig, velocity, yaw_rate);

	bool pass = true;

	// check that all particles have same state
	Particle p = pf.particles[0];
	for (int i=1; i < pf.particles.size(); i++) {
		if (p.x != pf.particles[i].x) {
			cout << "x coordinate does not match, " << p.x << "!=" << pf.particles[i].x << endl;
			pass = false;
		}
		if (p.y != pf.particles[i].y) {
			cout << "y coordinate does not match, " << p.y << "!=" << pf.particles[i].x << endl;
			pass = false;
		}
		if (p.theta != pf.particles[i].theta) {
			cout << "theta coordinate does not match, " << p.theta << "!=" << pf.particles[i].x << endl;
			pass = false;
		}
	}

	if (!equal(p.x, x + 1)) {
		cout << "x prediction is incorrect, " << p.x << "!=" << x + 1 << endl;
		pass = false;
	}
	if (!equal(p.y, y)) {
		cout << "y prediction is incorrect, " << p.y << "!=" << y << endl;
		pass = false;
	}
	if (!equal(p.theta, theta)) {
		cout << "theta prediction is incorrect, " << p.theta << "!=" << theta << endl;
		pass = false;
	}

	if (pass) {
		cout << "\t Test Passed!" << endl;
	}
	else {
		cout << "\t Test Failed!" << endl;
	}
}

void ParticleFilter_prediction_zero_yaw_rate_y_only() {

	cout << "Test ParticleFilter_prediction_zero_yaw_rate_y_only" << endl;

	ParticleFilter pf;

	double x = 1;
	double y = 1;
	double theta = M_PI/2;
	double sig [3] = {0.0, 0.0, 0.0};

	pf.init(x, y, theta, sig);
	
	double dt = 1;
	double velocity = 1;
	double yaw_rate = 0;

	pf.prediction(dt, sig, velocity, yaw_rate);

	bool pass = true;

	// check that all particles have same state
	Particle p = pf.particles[0];
	for (int i=1; i < pf.particles.size(); i++) {
		if (p.x != pf.particles[i].x) {
			cout << "x coordinate does not match, " << p.x << "!=" << pf.particles[i].x << endl;
			pass = false;
		}
		if (p.y != pf.particles[i].y) {
			cout << "y coordinate does not match, " << p.y << "!=" << pf.particles[i].x << endl;
			pass = false;
		}
		if (p.theta != pf.particles[i].theta) {
			cout << "theta coordinate does not match, " << p.theta << "!=" << pf.particles[i].x << endl;
			pass = false;
		}
	}

	if (!equal(p.x, x)) {
		cout << "x prediction is incorrect, " << p.x << "!=" << x << endl;
		pass = false;
	}
	if (!equal(p.y, y + 1)) {
		cout << "y prediction is incorrect, " << p.y << "!=" << y + sin(theta) << endl;
		pass = false;
	}
	if (!equal(p.theta, theta)) {
		cout << "theta prediction is incorrect, " << p.theta << "!=" << theta << endl;
		pass = false;
	}

	if (pass) {
		cout << "\t Test Passed!" << endl;
	}
	else {
		cout << "\t Test Failed!" << endl;
	}
}

void dist_computes_euclidean_distance() {

	cout << "Test dist_computes_euclidean_distance" << endl;

	bool pass = true;
	if (dist(1, 1, 1, 1) > 0.0001) {
		cout << "Distance between same point was nonzero." << endl;
		pass = false;
	}
	if (fabs(dist(0, 0, 1, 1) - sqrt(2)) > 0.0001) {
		cout << "Distance between two points is incorrect." << endl;
		pass = false;
	}

	if (pass) {
		cout << "\t Test Passed!" << endl;
	}
	else {
		cout << "\t Test Failed!" << endl;
	}
}

void ParticleFilter_dataAssociation_perfect_match() {

	cout << "Test ParticleFilter_dataAssociation_perfect_match" << endl;

	vector<LandmarkObs> predictions;
	for (int i=0; i < 3; i++) {
		LandmarkObs pred = {i, (double) i, 0};
		predictions.push_back(pred);
	}

	vector<LandmarkObs> observations;
	for (int i=0; i < 3; i++) {
		LandmarkObs obs = {-1, (double) i, 0};
		observations.push_back(obs);
	}

	ParticleFilter pf;
	pf.dataAssociation(predictions, observations);

	bool pass = true;
	for (int i=0; i < 3; i++) {
		if (predictions[i].id != observations[i].id) {
			cout << "Wrong id has been assigned to observation" << endl;
			pass = false;
		}
	}

	if (pass) {
		cout << "\t Test Passed!" << endl;
	}
	else {
		cout << "\t Test Failed!" << endl;
	}
}

void ParticleFilter_dataAssociation_many_to_one_match() {

	cout << "Test ParticleFilter_dataAssociation_many_to_one_match" << endl;

	vector<LandmarkObs> predictions;
	for (int i=0; i < 3; i++) {
		LandmarkObs pred = {i, (double) i, 0};
		predictions.push_back(pred);
	}

	vector<LandmarkObs> observations;
	for (int i=0; i < 3; i++) {
		LandmarkObs obs = {-1, 0, 0};
		observations.push_back(obs);
	}

	ParticleFilter pf;
	pf.dataAssociation(predictions, observations);

	bool pass = true;
	for (int i=0; i < 3; i++) {
		if (observations[i].id != 0) {
			cout << "Wrong id has been assigned to observation" << endl;
			pass = false;
		}
	}

	if (pass) {
		cout << "\t Test Passed!" << endl;
	}
	else {
		cout << "\t Test Failed!" << endl;
	}
}

int main() {
	ParticleFilter_init();
	ParticleFilter_prediction_zero_yaw_rate_x_only();
	ParticleFilter_prediction_zero_yaw_rate_y_only();
	dist_computes_euclidean_distance();
	ParticleFilter_dataAssociation_perfect_match();
	ParticleFilter_dataAssociation_many_to_one_match();
	return 0;
}