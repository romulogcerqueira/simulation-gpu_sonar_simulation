/*
 * SonarScan.hpp
 *
 *  Created on: Jul 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _SIMSONAR_HPP_
#define _SIMSONAR_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <base/samples/SonarBeam.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/Angle.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class SimSonar {

public:

	SimSonar():
		_number_of_bins(500),
		_speed_of_sound(1500.0f),
		_range(100.0f),
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

	const float min_range = 1.0f;
	const float max_range = 300.0f;

private:
	float sigmoid (float value);

protected:
	int _number_of_bins;

	float _speed_of_sound;
	float _range;
	float _beamwidth_horizontal;
	float _beamwidth_vertical;
};

} // end namespace gpu_sonar_simulation

#endif
