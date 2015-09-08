/*
 * MultibeamSonar.hpp
 *
 *  Created on: Jul 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _MULTIBEAMSONAR_HPP_
#define _MULTIBEAMSONAR_HPP_

#include "CommonSonar.hpp"

namespace gpu_sonar_simulation {
class MultibeamSonar : public CommonSonar {

public:

	MultibeamSonar():
		CommonSonar(),
		_start_bearing(-60.0f),
		_angular_resolution(1.0f),
		_number_of_beams(256),
		_pixels_per_beam(2)
	{
		_beamwidth_horizontal = 120.0f;
		_beamwidth_vertical = 20.0f;
		_viewer = cv::Mat::zeros(500*2, 500*1.2, CV_8UC3);
	};

	base::samples::SonarScan simulateSonarScan (std::vector<uint8_t> data);
	void plotSonarData(base::samples::SonarScan sonar, float range, int gain);
	std::vector<uint8_t> codeSonarData(cv::Mat3f cv_image);
	void testDistanceShader(cv::Mat3f image, double maxRange, double maxAngleX);

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

	float getAngularResolution() const {
		return _angular_resolution;
	}

	void setAngularResolution(float angularResolution) {
		_angular_resolution = angularResolution;
	}

	float getStartBearing() const {
		return _start_bearing;
	}

	void setStartBearing(float startBearing) {
		_start_bearing = startBearing;
	}

private:
	float _start_bearing;
	float _angular_resolution;

	int _number_of_beams;
	int _pixels_per_beam;
};

} // end namespace gpu_sonar_simulation

#endif
