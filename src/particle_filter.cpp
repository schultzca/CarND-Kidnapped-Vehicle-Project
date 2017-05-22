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
#include <cmath>
#include "Eigen/Dense"

#include "particle_filter.h"

 using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	
	// Initialize the number of particles if it hasn't been already. 
	if (num_particles == 0) {
		num_particles = 1000;
	}

	// Change vector size to hold number of particles.
	particles.resize(num_particles);

	// Generate particles using normal distribution
	default_random_engine gen;
	normal_distribution<double> x_dist (x, std[0]);
	normal_distribution<double> y_dist (y, std[1]);
	normal_distribution<double> theta_dist (theta, std[2]); 
	for (int i=0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = x_dist(gen);
		p.y = y_dist(gen);
		p.theta = theta_dist(gen);
		particles[i] = p;
	}

	// initialize weights to 1
	weights = vector<double> (num_particles, 1.0);
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	
	default_random_engine gen;
	normal_distribution<double> x_dist (0, std_pos[0]);
	normal_distribution<double> y_dist (0, std_pos[1]);
	normal_distribution<double> theta_dist (0, std_pos[2]);
	for (int i=0; i < num_particles; i++) {

		Particle p = particles[i];
		
		// straight line model
		if (fabs(yaw_rate) < 0.001) {
			p.x = p.x + velocity * cos(p.theta) * delta_t;
			p.y = p.y + velocity * sin(p.theta) * delta_t;
		} 
		// constant turn rate model
		else {
			p.x = p.x + (velocity/yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
			p.y = p.y + (velocity/yaw_rate) * (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta));
			p.theta = p.theta + yaw_rate * delta_t;
		}

		// add noise
		p.x += x_dist(gen);
		p.y += y_dist(gen);
		p.theta += theta_dist(gen);
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i=0; i < observations.size(); i++) {

		// current observation
		LandmarkObs obs = observations[i];
		
		// set minimum distance to value greater than sensor range
		double min_dist = 1.0e10;
		for (int j=0; j < predicted.size(); j++) {

			LandmarkObs pred = predicted[j];

			// compute euclidian distance
			double dist = euclidianDistance(pred.x, obs.x, pred.y, obs.y);

			if (dist < min_dist) {

				// update observation id
				obs.id = j;

				// set new minimum distance
				min_dist = dist;
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
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

	// Sum of weights
	double weights_sum = 0;
	
	for (int i=0; i < particles.size(); i++) {

		Particle p = particles[i];

		// convert observations to map coordinates
		vector<LandmarkObs> observations_in_map;
		for (int j=0; j < observations.size(); j++) {

			LandmarkObs tmp;
			
			LandmarkObs obs = observations[j];

			// apply rotation and translation to observation
			tmp.x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
			tmp.y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;

			observations_in_map.push_back(tmp);
		}

		// filter out landmarks outside of sensor range
		vector<LandmarkObs> landmarks_in_range;
		for (int j=0; j < map_landmarks.landmark_list.size(); j++) {

			Map::single_landmark_s lm = map_landmarks.landmark_list[j];

			LandmarkObs tmp;

			// add point if distance from particle is less than or equal to sensor range
			if (euclidianDistance(lm.x_f, p.x, lm.y_f, p.y) <= sensor_range) {
				tmp = {lm.id_i, lm.x_f, lm.y_f};
				landmarks_in_range.push_back(tmp);
			}
		}

		// associate landmarks with observations
		dataAssociation(landmarks_in_range, observations_in_map);

		// compute probability using multivariate gaussian
		double weight = 1.0;
		for (int j=0; j < observations_in_map.size(); j++) {

			LandmarkObs observed = observations_in_map[j];

			LandmarkObs predicted = landmarks_in_range[observed.id];

			weight = weight * multivariateGaussian(predicted, observed, std_landmark);
		}
		weights[i] = weight;
		weights_sum += weight;
	}

	// Normalize weights
	for (int i=0; i < particles.size(); i++) {
		weights[i] = weights[i] / weights_sum;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
}

void ParticleFilter::write(string filename) {
	// You don't need to modify this file.
	ofstream dataFile;
	dataFile.open(filename, ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

double ParticleFilter::euclidianDistance(double x1, double x2, double y1, double y2) {
	return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

double ParticleFilter::multivariateGaussian(LandmarkObs predicted, LandmarkObs observed, double sigma_landmark[2]) {
	// Unpack coordiantes
	double x = observed.x;
	double y = observed.y;
	double mu_x = predicted.x;
	double mu_y = predicted.y;

	// Unpack landmark variance values
	double x_sig = sigma_landmark[0];
	double y_sig = sigma_landmark[1];

	// Bivariate normal distribution with zero correlation assumption
	const double pi = 3.14159265359;
	return (1 / (2 * pi * x_sig * y_sig)) * std::exp(-0.5 * (pow(x - mu_x, 2) / pow(x_sig, 2) + pow(y - mu_y, 2) / pow(y_sig, 2)));
}


