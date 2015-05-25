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
		_bearing(0.0),
		_number_of_bins(500)
		{};

	cv::Mat decodeShaderImage(cv::Mat raw_image);
	std::vector<uint8_t> getPingData(cv::Mat raw_intensity);
	base::samples::SonarBeam simulateSonarBeam (std::vector<uint8_t> data, float step_angle);


	int getNumberOfBins() const {
		return _number_of_bins;
	}

	void setNumberOfBins(int numberOfBins) {
		_number_of_bins = numberOfBins;
	}

private:
	float sigmoid (float value);
	std::vector<uint8_t> applyDynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span);

	double _bearing;
	int _number_of_bins;
};

} // end namespace gpu_sonar_simulation

#endif
