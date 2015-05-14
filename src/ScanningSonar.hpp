#ifndef _SCANNINGSONAR_HPP_
#define _SCANNINGSONAR_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <base/samples/SonarBeam.hpp>
#include <base/Angle.hpp>
#include <base/Time.hpp>

#include "SonarConfig.hpp"

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class ScanningSonar {

public:
	ScanningSonar():
		_number_of_bins(0),
		_ad_interval(0),
		_bearing(0),
		_motor_step_angle_size(0),
		_speed_of_sound(1500.0),
		_max_distance(75.0),
		_config()
	{};

	void setup(SonarConfig config);
	cv::Mat decodeRawImage(cv::Mat raw_image);
	std::vector<uint8_t> getPingIntensity(cv::Mat raw_intensity);
	std::vector<uint8_t> dynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span);
	base::samples::SonarBeam simulateSonarBeam(std::vector<uint8_t> data);
	double sigmoid(double x);
	double halfGaussian(double x);


	double getMaxDistance()
	{
		return _max_distance;
	}

private:
	int _ad_interval;

	uint16_t _number_of_bins;
	uint16_t _bearing;

	uint8_t _motor_step_angle_size;

	double _speed_of_sound;
	double _max_distance;

	SonarConfig _config;
};

} // end namespace gpu_sonar_simulation

#endif
