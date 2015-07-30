/*
 * MultibeamSonar.hpp
 *
 *  Created on: Jul 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _MULTIBEAMSONAR_HPP_
#define _MULTIBEAMSONAR_HPP_

#include "SimSonar.hpp"

namespace gpu_sonar_simulation {
class MultibeamSonar : public SimSonar {

public:

	MultibeamSonar():
		SimSonar(),
		_start_bearing(-60.0f),
		_angular_resolution(1.0f),
		_number_of_beams(256),
		_pixels_per_beam(4)
	{
		_beamwidth_horizontal = 120.0f;
		_beamwidth_vertical = 20.0f;
	};

	base::samples::SonarScan simulateSonarScan (std::vector<uint8_t> data);
	std::vector<cv::Mat> splitShaderImage(cv::Mat cv_image);

	int getNumberOfBeams() const {
		return _number_of_beams;
	}

	void setNumberOfBeams(int numberOfBeams) {
		_number_of_beams = numberOfBeams;
	}

	int getPixelsPerBeam() const {
		return _pixels_per_beam;
	}

	void setPixelsPerBeam(int pixelsPerBeam) {
		_pixels_per_beam = pixelsPerBeam;
	}

private:
	float _start_bearing;
	float _angular_resolution;

	int _number_of_beams;
	int _pixels_per_beam;
};

} // end namespace gpu_sonar_simulation

#endif
