#include "ScanSonar.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

// Accepts the input value x then returns it's sigmoid value in float
float ScanSonar::sigmoid(float x) {

	float l = 1, k = 18, x0 = 0.666666667;
	float exp_value;

	// Exponential calculation
	exp_value = exp( (double)(-k * (x - x0)));

	// Final sigmoid value
	return (l / (1 + exp_value));
}


// Receive shader image (normal and depth matrixes) and convert to get
// the bins intensities
cv::Mat ScanSonar::decodeShaderImage(cv::Mat raw_image) {

	if (raw_image.type() != CV_32FC3){
		std::cout << "Invalid shader image format!" << std::endl;
		exit(0);
	}

	float interval = 1.0 / (float) _number_of_bins;

	cv::Mat bins_depth;
	cv::Mat bins_normal = cv::Mat::zeros(_number_of_bins, 1, CV_32F);

	// calculate depth histogram
	std::vector<cv::Mat> shader;
	split(raw_image, shader);
	shader[1] = 1.0 - shader[1];

	float range[] = {0.0f, 1.0f};
	const float *hist_range = {range};

	cv::calcHist(&shader[1], 1, 0, cv::Mat(), bins_depth, 1, &_number_of_bins, &hist_range);
	bins_depth.convertTo(bins_depth, CV_32S);

	// calculate bins intensities using normal values, depth histogram and sigmoid function
	for(int i=0; i<raw_image.rows; i++)
		for(int j=0; j<raw_image.cols; j++) {
			int id_bin = shader[1].at<float>(i,j) / interval;
			if (id_bin == _number_of_bins) id_bin--;
			if (shader[0].at<float>(i,j) > 0)
				bins_normal.at<float>(id_bin) += (1.0 / (float) bins_depth.at<int>(id_bin)) * sigmoid(shader[0].at<float>(i,j));
		}

	return bins_normal;
}


// Calculate ping intensity in 8-bit format data
std::vector<uint8_t> ScanSonar::getPingData(cv::Mat raw_intensity) {

	std::vector<uint8_t> ping_intensity;
	raw_intensity.convertTo(ping_intensity, CV_8U, 255);

	return ping_intensity;
}

// Calculate the sample time period that is applied to the received Sonar echo signal
uint16_t ScanSonar::getADInterval() {
	float travel_time = _range * 2.0 / _speed_of_sound;
	float sample_time = travel_time / (float) _number_of_bins;
	uint8_t ad_interval = (uint8_t) ((sample_time / (640 * 1e-9)) + 0.5);

	return ad_interval;
}


// Simulate a base::samples::SonarBeam data and update the sonar head position
base::samples::SonarBeam ScanSonar::simulateSonarBeam (std::vector<uint8_t> data) {
	base::samples::SonarBeam beam;

	beam.time = base::Time::now();
	beam.bearing = base::Angle::fromDeg(-_bearing + 90.0f);
	beam.sampling_interval = 640.0 * 1e-9 * getADInterval();
	beam.speed_of_sound = _speed_of_sound;
	beam.beamwidth_horizontal = base::Angle::deg2Rad(_beamwidth_horizontal);
	beam.beamwidth_vertical = base::Angle::deg2Rad(_beamwidth_vertical);
	beam.beam = data;

	// if ping_pong is false, the sonar scans from _left_limit to _right_limit in loop
	if (!_ping_pong_mode)
	{
		_bearing += _step_angle;

		if(_bearing > _right_limit)
			_bearing = _left_limit;
	}

	// otherwise, the sonar scans from _left_limit to _right_limit and vice versa in loop
	else
	{
		// it scans from _left_limit to _right_limit
		if(!_reverse_scan) {
			_bearing += _step_angle;

			if(_bearing > _right_limit){
				_bearing = _right_limit;
				_reverse_scan = true;
			}
		}

		// it scans from _right_limit to _left_limit
		else
		{
			_bearing -= _step_angle;

			if(_bearing < _left_limit) {
				_bearing = _left_limit;
				_reverse_scan = false;
			}
		}
	}

	return beam;

}

}
