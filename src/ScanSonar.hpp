/*
 * SonarScan.hpp
 *
 *  Created on: Apr 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _SCANSONAR_HPP_
#define _SCANSONAR_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <base/samples/SonarBeam.hpp>
#include <base/Angle.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class ScanSonar {

public:
	ScanSonar():
		_number_of_bins(500),
		_bearing(0.0f),
		_left_limit(0.0f),
		_right_limit(360.0f),
		_speed_of_sound(1500.0f),
		_range(50.0f),
		_step_angle(1.8f),
		_beamwidth_horizontal(3.0f),
		_beamwidth_vertical(35.0f),
		_ping_pong_mode(false),
		_reverse_scan(false)
		{};

	cv::Mat decodeShaderImage(cv::Mat raw_image);
	std::vector<uint8_t> getPingData(cv::Mat raw_intensity);
	uint16_t getADInterval();
	base::samples::SonarBeam simulateSonarBeam (std::vector<uint8_t> data);


	int getNumberOfBins() const {
		return _number_of_bins;
	}

	void setNumberOfBins(int numberOfBins) {
		_number_of_bins = numberOfBins;
	}

	bool isReverseScan() const {
		return _reverse_scan;
	}

	void setReverseScan(bool reverseScan) {
		_reverse_scan = reverseScan;
	}

	float getLeftLimit() const {
		return _left_limit;
	}

	void setLeftLimit(float leftLimit) {
		_left_limit = leftLimit;
	}

	bool isPingPongMode() const {
		return _ping_pong_mode;
	}

	void setPingPongMode(bool pingPongMode) {
		_ping_pong_mode = pingPongMode;
	}

	float getRightLimit() const {
		return _right_limit;
	}

	void setRightLimit(float rightLimit) {
		_right_limit = rightLimit;
	}

	float getRange() const {
		return _range;
	}

	void setRange(float range) {
		_range = range;
	}

	float getStepAngle() const {
		return _step_angle;
	}

	void setStepAngle(float stepAngle) {
		_step_angle = stepAngle;
	}

	float getBeamwidthHorizontal() const {
		return _beamwidth_horizontal;
	}

	void setBeamwidthHorizontal(float beamwidthHorizontal) {
		_beamwidth_horizontal = beamwidthHorizontal;
	}

	float getBeamwidthVertical() const {
		return _beamwidth_vertical;
	}

	void setBeamwidthVertical(float beamwidthVertical) {
		_beamwidth_vertical = beamwidthVertical;
	}

	const float min_range = 0.5f;
	const float max_range = 75.0f;

private:
	float sigmoid (float value);

	int _number_of_bins;

	float _bearing;
	float _left_limit;
	float _right_limit;
	float _speed_of_sound;
	float _range;
	float _step_angle;
	float _beamwidth_horizontal;
	float _beamwidth_vertical;

	bool _ping_pong_mode;
	bool _reverse_scan;
};

} // end namespace gpu_sonar_simulation

#endif
