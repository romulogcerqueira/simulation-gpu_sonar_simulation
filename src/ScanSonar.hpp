/*
 * SonarScan.hpp
 *
 *  Created on: Apr 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _SCANSONAR_HPP_
#define _SCANSONAR_HPP_

#include "CommonSonar.hpp"

namespace gpu_sonar_simulation {
class ScanSonar : public CommonSonar {

public:

	ScanSonar():
		CommonSonar(),
		_bearing(0),
		_start_angle(0),
		_end_angle(M_PI*2),
		_step_angle(base::Angle::deg2Rad(1.8)),
		_ping_pong_mode(false),
		_reverse_scan(false)
	{
		_beamwidth_horizontal = 3.0f;
		_beamwidth_vertical = 35.0f;
	};

	base::samples::SonarBeam simulateSonarBeam (const std::vector<uint8_t>& data);

	bool isReverseScan() const {
		return _reverse_scan;
	}

	void setReverseScan(bool reverseScan) {
		_reverse_scan = reverseScan;
	}

	bool isPingPongMode() const {
		return _ping_pong_mode;
	}

	void setPingPongMode(bool pingPongMode) {
		_ping_pong_mode = pingPongMode;
	}

	double getBearing() const {
		return _bearing;
	}

	void setBearing(double bearing) {
		_bearing = bearing;
	}

	double getEndAngle() const {
		return _end_angle;
	}

	void setEndAngle(double endAngle) {
		_end_angle = endAngle;
	}

	double getStartAngle() const {
		return _start_angle;
	}

	void setStartAngle(double startAngle) {
		_start_angle = startAngle;
	}

	double getStepAngle() const {
		return _step_angle;
	}

	void setStepAngle(double stepAngle) {
		_step_angle = stepAngle;
	}

private:
	double _bearing;
	double _start_angle;
	double _end_angle;
	double _step_angle;

	bool _ping_pong_mode;
	bool _reverse_scan;
};

} // end namespace gpu_sonar_simulation

#endif
