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
		number_of_bins(500),
		bearing(-90.0),
		left_limit(-90.0),
		right_limit(90.0),
		ping_pong_mode(true),
		reverse_scan(false)
		{};

	cv::Mat decodeShaderImage(cv::Mat raw_image);
	std::vector<uint8_t> getPingData(cv::Mat raw_intensity);
	base::samples::SonarBeam simulateSonarBeam (std::vector<uint8_t> data, float step_angle);


	int getNumberOfBins() const {
		return number_of_bins;
	}

	void setNumberOfBins(int numberOfBins) {
		number_of_bins = numberOfBins;
	}

	bool isReverseScan() const {
		return reverse_scan;
	}

	void setReverseScan(bool reverseScan) {
		reverse_scan = reverseScan;
	}

	double getLeftLimit() const {
		return left_limit;
	}

	void setLeftLimit(double leftLimit) {
		left_limit = leftLimit;
	}

	bool isPingPongMode() const {
		return ping_pong_mode;
	}

	void setPingPongMode(bool pingPongMode) {
		ping_pong_mode = pingPongMode;
	}

	double getRightLimit() const {
		return right_limit;
	}

	void setRightLimit(double rightLimit) {
		right_limit = rightLimit;
	}

private:
	float sigmoid (float value);

	int number_of_bins;
	double bearing;
	double left_limit;
	double right_limit;
	bool ping_pong_mode;
	bool reverse_scan;
};

} // end namespace gpu_sonar_simulation

#endif
