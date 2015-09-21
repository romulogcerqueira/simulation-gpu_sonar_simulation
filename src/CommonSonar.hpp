/*
 * SonarScan.hpp
 *
 *  Created on: Jul 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _COMMONSONAR_HPP_
#define _COMMONSONAR_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <base/samples/SonarBeam.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/Angle.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class CommonSonar {

public:

	CommonSonar():
		_number_of_bins(500),
		_gain(0.7f),
		_speed_of_sound(1500.0f),
		_range(50.0f),
		_beamwidth_horizontal(0.0f),
		_beamwidth_vertical(0.0f)
	{};

	cv::Mat decodeShaderImage(cv::Mat raw_image);
	std::vector<uint8_t> getPingData(cv::Mat raw_intensity);
	double getSamplingInterval();

	int getNumberOfBins() const {
		return _number_of_bins;
	}

	void setNumberOfBins(int numberOfBins) {
		_number_of_bins = numberOfBins;
	}

	float getRange() const {
		return _range;
	}

	void setRange(float range) {
		_range = range;
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

	float getSpeedOfSound() const {
		return _speed_of_sound;
	}

	void setSpeedOfSound(float speedOfSound) {
		_speed_of_sound = speedOfSound;
	}

	float getGain() const {
		return _gain;
	}

	void setGain(float gain) {
		_gain = gain;
	}

private:
	float sigmoid (float value);

protected:
	int _number_of_bins;
	float _gain;

	float _speed_of_sound;
	float _range;
	float _beamwidth_horizontal;
	float _beamwidth_vertical;
};

} // end namespace gpu_sonar_simulation

#endif
