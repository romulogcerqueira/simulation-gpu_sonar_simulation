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
		_number_of_beams(256),
		_pixels_per_beam(2),
		_start_bearing(base::Angle::deg2Rad(-55.0))
	{
		_beamwidth_horizontal = 110.0f;
		_beamwidth_vertical = 20.0f;
	};

	base::samples::SonarScan simulateSonarScan(const std::vector<uint8_t>& data);
	std::vector<uint8_t> codeSonarData(const cv::Mat3f& cv_image);

	unsigned int getNumberOfBeams() const {
		return _number_of_beams;
	}

	void setNumberOfBeams(unsigned int numberOfBeams) {
		_number_of_beams = numberOfBeams;
	}

	unsigned int getPixelsPerBeam() const {
		return _pixels_per_beam;
	}

	void setPixelsPerBeam(unsigned int pixelsPerBeam) {
		_pixels_per_beam = pixelsPerBeam;
	}

	double getStartBearing() const {
		return _start_bearing;
	}

	void setStartBearing(double startBearing) {
		_start_bearing = startBearing;
	}

private:
	unsigned int _number_of_beams;
	unsigned int _pixels_per_beam;

	double _start_bearing;
};

} // end namespace gpu_sonar_simulation

#endif
