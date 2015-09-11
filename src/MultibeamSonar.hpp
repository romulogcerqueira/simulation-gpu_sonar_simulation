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
		_start_bearing(base::Angle::deg2Rad(-60.0)),
		_angular_resolution(base::Angle::deg2Rad(1.0)),
		_number_of_beams(256),
		_pixels_per_beam(2)
	{
		_beamwidth_horizontal = 120.0f;
		_beamwidth_vertical = 20.0f;
	};

	base::samples::SonarScan simulateSonarScan (std::vector<uint8_t> data);
	std::vector<uint8_t> codeSonarData(cv::Mat3f cv_image);

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

	double getAngularResolution() const {
		return _angular_resolution;
	}

	void setAngularResolution(double angularResolution) {
		_angular_resolution = angularResolution;
	}

	double getStartBearing() const {
		return _start_bearing;
	}

	void setStartBearing(double startBearing) {
		_start_bearing = startBearing;
	}

private:
	double _start_bearing;
	double _angular_resolution;

	int _number_of_beams;
	int _pixels_per_beam;
};

} // end namespace gpu_sonar_simulation

#endif
