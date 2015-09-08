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
		_bearing(0.0f),
		_start_angle(0.0f),
		_end_angle(360.0f),
		_step_angle(1.8f),
		_ping_pong_mode(false),
		_reverse_scan(false)
	{
		_beamwidth_horizontal = 3.0f;
		_beamwidth_vertical = 35.0f;
		_viewer = cv::Mat(500 * 2 + 10, 500 * 2 + 10, CV_8UC3);
	};

	base::samples::SonarBeam simulateSonarBeam (std::vector<uint8_t> data);
	void plotSonarData(base::samples::SonarBeam sonar, float range, int gain);

	bool isReverseScan() const {
		return _reverse_scan;
	}

	void setReverseScan(bool reverseScan) {
		_reverse_scan = reverseScan;
	}

	float getStartAngle() const {
		return _start_angle;
	}

	void setStartAngle(float startAngle) {
		_start_angle = startAngle;
	}

	bool isPingPongMode() const {
		return _ping_pong_mode;
	}

	void setPingPongMode(bool pingPongMode) {
		_ping_pong_mode = pingPongMode;
	}

	float getEndAngle() const {
		return _end_angle;
	}

	void setEndAngle(float endAngle) {
		_end_angle = endAngle;
	}

	float getStepAngle() const {
		return _step_angle;
	}

	void setStepAngle(float stepAngle) {
		_step_angle = stepAngle;
	}

private:
	float _bearing;
	float _start_angle;
	float _end_angle;
	float _step_angle;

	bool _ping_pong_mode;
	bool _reverse_scan;
};

} // end namespace gpu_sonar_simulation

#endif
