#include "SonarSim.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

SonarSim::SonarSim() {
	_number_of_bins = 500;
	_range = 75.0;
	_speed_of_sound = 1500.0;
	_ad_low = 8;
	_ad_span = 81;
	_gain = 1.0;
}

SonarSim::~SonarSim() {
}

// Receive the depth and normal matrixes from the source image and
// return the maximum intensity for each bin
cv::Mat SonarSim::decodeRawImage(cv::Mat raw_image) {

	std::cout << "Decoding simulated image..." << std::endl;

	if (raw_image.type() != CV_32FC3) {
		std::cout << "Invalid image format!" << std::endl;
		exit(0);
	}

	float interval = 1.0 / (float) _number_of_bins;

	cv::Mat bin_depth = cv::Mat::zeros(_number_of_bins, 1, CV_32S);
	cv::Mat raw_intensity = cv::Mat::zeros(_number_of_bins, 1, CV_32F);


	// organize depth and normal values and generate the intensity for each bin
	for (int i = 0; i < raw_image.rows; i++) {
		for (int j = 0; j < raw_image.cols; j++) {
			int id_bin = (1.0 - raw_image.at<Vec3f>(i, j)[0]) / interval;
			if (id_bin == _number_of_bins)
				id_bin--;
			bin_depth.at<int>(id_bin)++;

			// sum of intensities (using the sum/sigmoid/half gaussian of each normal)
			raw_intensity.at<float>(id_bin) += raw_image.at<Vec3f>(i, j)[1];
//			raw_intensity.at<float>(id_bin) += sigmoid(raw_image.at<Vec3f>(i, j)[1]);
//			raw_intensity.at<float>(id_bin) += halfGaussian(raw_image.at<Vec3f>(i, j)[1]);
		}
	}

	// calculate the mean of each intensity bin
	for (int i = 0; i < _number_of_bins; i++)
		raw_intensity.at<float>(i) /= (float) bin_depth.at<int>(i);

//	std::cout << "INTENSITY: " << raw_intensity << std::endl;

	return raw_intensity;
}

// Calculate the maximum intensity of one ping (8-bit format) by normal histogram
std::vector<uint8_t> SonarSim::getPingIntensity(cv::Mat raw_intensity) {

	std::vector<uint8_t> data;

	// convert to SonarScan::data format (8-bit)
	std::cout << "Simulating beam data..." << std::endl;
	raw_intensity.convertTo(data, CV_8U, 255);

	return data;
}

// Accepts the input value x then returns it's sigmoid value in float
float SonarSim::sigmoid(float x) {

	float l = 1, k = 30, x0 = 0.7777;
	float exp_value, return_value;

	exp_value = exp((double) -k * (x - x0));
	return_value = l / (1 + exp_value);

	return return_value;
}

// Accepts the input value x then returns it's half gaussian value in float
float SonarSim::halfGaussian(float x) {

	float a = 1.0, b = 1.0, c = 5.0;
	float exp_value, return_value;

	exp_value = exp((double) -(pow(x - b, 2) / 2 * pow(c, 2)));

	return_value = a * exp_value;

	return return_value;
}

// Calculate the sample time period that is applied to the received Sonar echo signal
uint16_t SonarSim::getADInterval() {
	float travel_time = _range * 2.0 / _speed_of_sound;
	float sample_time = travel_time / (float) _number_of_bins;
	uint8_t ad_interval = (uint8_t) ((sample_time / (640 * 1e-9)) + 0.5);

	return ad_interval;
}

// Generate simulated base::samples::SonarScan data
base::samples::SonarScan SonarSim::createSimSonarData(
		std::vector<uint8_t> data) {

	std::cout << "Generating simulated sonar data..." << std::endl;

	base::samples::SonarScan sonar;
	uint16_t ad_interval = getADInterval();

	// Micron DST
	sonar.time = base::Time::now();
	sonar.data = data;
	sonar.time_beams = std::vector<base::Time>();
	sonar.number_of_beams = 1;
	sonar.number_of_bins = _number_of_bins;
	sonar.start_bearing = base::Angle::fromDeg(0.0);
	sonar.angular_resolution = base::Angle::fromDeg(1.8);
	sonar.sampling_interval = ((640.0 * ad_interval) * 1e-9);
	sonar.beamwidth_horizontal = base::Angle::fromDeg(3.0);
	sonar.beamwidth_vertical = base::Angle::fromDeg(90.0);
	sonar.speed_of_sound = _speed_of_sound;

	return sonar;
}

}
