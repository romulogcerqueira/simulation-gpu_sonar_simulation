#ifndef _SONARSIM_HPP_
#define _SONARSIM_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <base/samples/SonarScan.hpp>
#include <base/Time.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class SonarSim {
public:
	SonarSim();
	~SonarSim();

	float sigmoid(float x);
	uint16_t getADInterval();
	cv::Mat decodeRawImage(cv::Mat raw_image);
	std::vector<uint8_t> getPingIntensity(cv::Mat raw_intensity);
	base::samples::SonarScan createSimSonarData(std::vector<uint8_t> beam);

	uint16_t getNumberOfBins() const {
		return _number_of_bins;
	}

	void setNumberOfBins(uint16_t numberOfBins) {
		_number_of_bins = numberOfBins;
	}

	float getRange() const {
		return _range;
	}

	void setRange(float range) {
		_range = range;
	}

private:
	uint16_t _number_of_bins;
	uint8_t _ad_low;
	uint8_t _ad_span;
	float _range;
	float _speed_of_sound;
};

} // end namespace gpu_sonar_simulation

#endif
