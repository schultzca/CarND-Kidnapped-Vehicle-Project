/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	// state specific random number generators
	default_random_engine generator;

	// normal random variables for each state
	normal_distribution<double> x_normal_distribution(x, std[0]);
	normal_distribution<double> y_normal_distribution(y, std[1]);
	normal_distribution<double> theta_normal_distribution(theta, std[2]);

	cout << "num_particles: " << num_particles << endl;

	// generate particles
	particles.resize(num_particles);
	for (int i=0; i < num_particles; i++) {
		
		// create and initialize particle
		Particle p;
		p.id = i;
		p.x = x_normal_distribution(generator);
		p.y = y_normal_distribution(generator);
		p.theta = theta_normal_distribution(generator);
		p.weight = -1;

		particles[i] = p;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find normal_distribution and default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// state specific random number generators
	default_random_engine generator;

	// normal random variables for each state
	normal_distribution<double> x_noise_normal_distribution(0, std_pos[0]);
	normal_distribution<double> y_noise_normal_distribution(0, std_pos[1]);
	normal_distribution<double> theta_noise_normal_distribution(0, std_pos[2]);

	// update each particles position using velocity and yaw rate
	for (int i=0; i < particles.size(); i++) {
		
		Particle p = particles[i];
		
		// straight line motion
		if (fabs(yaw_rate) < 0.001) {
			p.x = p.x + velocity * delta_t * cos(p.theta);
			p.y = p.y + velocity * delta_t * sin(p.theta);
		}
		else {
			p.x = p.x + (velocity/yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
			p.y = p.y + (velocity/yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
			p.theta = p.theta + yaw_rate * delta_t;
		}

		// add gaussian noise
		p.x = p.x + x_noise_normal_distribution(generator);
		p.y = p.y + y_noise_normal_distribution(generator);
		p.theta = p.theta + theta_noise_normal_distribution(generator);
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	// iterate through each prediction
	for (int i=0; i < observations.size(); i++) {
		
		LandmarkObs obs = observations[i];

		// iterate through each observations
		double distance = 1000000;
		for (int j=0; j < predicted.size(); j++) {

			LandmarkObs pred = predicted[j];

			// compute euclidian distance to prediction
			double d = dist(pred.x, pred.y, obs.x, obs.y);
			
			// assign id to prediction if distance is minimum
			if (d < distance) {
				obs.id = pred.id;
				distance = d;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	// iterate through particles
	for (int i=0; i < particles.size(); i++) {

		Particle p = particles[i];

		// iterate through landmarks and filter landmarks out of sensor range
		vector<LandmarkObs> landmarks;
		for (int j=0; j < map_landmarks.landmark_list.size(); j++) {

			int id = map_landmarks.landmark_list[j].id_i;
			double x = map_landmarks.landmark_list[j].x_f;
			double y = map_landmarks.landmark_list[j].y_f;

			// store landmarks that are in sensor range
			if (dist(p.x, p.y, x, y) < sensor_range) {
				
				LandmarkObs obs;
				obs.id = id;
				obs.x = x;
				obs.y = y;

				landmarks.push_back(obs);
			}
		}

		// iterate through observations and transform to map coordinates
		vector<LandmarkObs> observations_t;
		for (int j=0; j < observations.size(); j++) {

			// original observation in vehicle coordinates
			LandmarkObs obs = observations[j];
			
			// transformed observation in map coordinates
			LandmarkObs obs_t;
			obs_t.id = obs.id;
			obs_t.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
			obs_t.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);

			// store transformed observation
			observations_t.push_back(obs_t);
		}

		// associate observations with landmarks
		dataAssociation(landmarks, observations_t);

		// compute particle weight and assign association
		p.weight = 1;
		for (int j=0; j < observations_t.size(); j++) {
			
			// observation position and landmark id
			int id = observations_t[j].id - 1;	// convert to vector index
			double x = observations_t[j].x;
			double y = observations_t[j].y;

			// associated landmark position
			double mx = map_landmarks.landmark_list[id].x_f;
			double my = map_landmarks.landmark_list[id].y_f;

			// landmark x standard deviation
			double xs = std_landmark[0];
			double xs2 = xs * xs;	//squared

			// landmark y standard deviation
			double ys = std_landmark[1];
			double ys2 = ys * ys;	//squared

			// x diff
			double dx = x - mx;
			double dx2 = dx * dx; // squared

			// y diff
			double dy = y - my;
			double dy2 = dy * dy; // squared

			double den = 2 * PI * xs * ys;
			
			// compute weight using multivariate gaussian
			p.weight = p.weight * exp(-(dx2/(2*xs2) + dy2/(2*ys2)))/den;
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
}

Particle ParticleFilter::SetAssociations(Particle particle, vector<int> associations, vector<double> sense_x, vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
		copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
		string s = ss.str();
		s = s.substr(0, s.length()-1);  // get rid of the trailing space
		return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
		copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
		string s = ss.str();
		s = s.substr(0, s.length()-1);  // get rid of the trailing space
		return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
		copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
		string s = ss.str();
		s = s.substr(0, s.length()-1);  // get rid of the trailing space
		return s;
}
