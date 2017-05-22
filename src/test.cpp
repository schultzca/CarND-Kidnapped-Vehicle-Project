#include <iostream>
#include "helper_functions.h"
#include "particle_filter.h"

bool multivariateGaussian_SimpleValues_Calculated() {
	double x[5] = {-2, -1, 0, 1, 2};
	double y[5] = {2, 1, 0, -1, -2};

	LandmarkObs predicted = {0, 0., 0.};

	double std[2] = {5., 2.};

	for (int i=0; i < 5; i++) {

		LandmarkObs observed = {0, x[i], y[i]};

		std::cout << ParticleFilter::multivariateGaussian(predicted, observed, std) << std::endl;
	}

	return true;
}

bool euclidianDistance_SimpleValues_Calculated() {
	double x[5] = {0,  0, 3,  0, 4};
	double y[5] = {0,  2, 0, -3, 0};

	double expected[5] = {0, 2, 3, 3, 4};

	bool pass = true;
	for (int i=0; i < 5; i++) {
		if (ParticleFilter::euclidianDistance(x[i], 0, y[i], 0) != expected[i]) {
			pass = false;
		}
	}
	return pass;
}

int main() {
	if (!multivariateGaussian_SimpleValues_Calculated()) {
		std::cout << "multivariateGaussian Tests failed." << std::endl;
	}
	else {
		std::cout << "multivariateGaussian Tests passed." << std::endl;
	}

	if (!euclidianDistance_SimpleValues_Calculated()) {
		std::cout << "euclidianDistance Tests failed." << std::endl;
	}
	else {
		std::cout << "euclidianDistance Tests passed." << std::endl;
	}
	return 0;
}