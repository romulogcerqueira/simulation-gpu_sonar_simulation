#include "SonarSim.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

SonarSim::SonarSim() {
	_number_of_bins = 1800;
	_range = 20.0;
	_speed_of_sound = 1500.0;
	_ad_low = 8;
	_ad_span = 81;
}

SonarSim::~SonarSim() {
}

// Receive the depth and normal matrixes from the source image and
// return the maximum intensity for each bin
std::vector<uint8_t> SonarSim::decodeRawImage(cv::Mat raw_image, int slices) {

	std::cout << "Decoding simulated image..." << std::endl;

	if (raw_image.type() != CV_32FC3) {
		std::cout << "Invalid image format!" << std::endl;
		exit(0);
	}

	float interval = 1.0 / (float) _number_of_bins;

	std::vector<int> bin_depth(_number_of_bins, 0);
	std::vector<std::vector<float> > bin_normal(_number_of_bins);

	cv::Mat histogram = cv::Mat::zeros(_number_of_bins, slices, CV_32S);

	// organize depth and normal values and generate the histogram
	for (int i = 0; i < raw_image.rows; i++) {
		for (int j = 0; j < raw_image.cols; j++) {
			int id_bin = raw_image.at<Vec3f>(i, j)[0] / interval;
			if (id_bin == _number_of_bins)
				id_bin--;
			bin_depth[id_bin]++;
			bin_normal[id_bin].push_back(raw_image.at<Vec3f>(i, j)[1]);
			int id_hist = (int) (raw_image.at<Vec3f>(i, j)[1] * slices);
			if (id_hist == slices)
				id_hist--;
			histogram.at<int>(id_bin, id_hist)++;}
		}

	return getPingIntensity(histogram);
}

// Calculate the maximum intensity of one ping (8-bit format) by normal histogram
std::vector<uint8_t> SonarSim::getPingIntensity(cv::Mat hist) {

	cv::Mat ping_intensity = cv::Mat::zeros(hist.rows, 1, CV_32F);
	std::vector<uint8_t> data;

	// calculate the maximum intensity of each bin
	for (int i = 0; i < hist.rows; i++) {
		cv::Point max_loc;
		cv::minMaxLoc(hist.row(i), NULL, NULL, NULL, &max_loc);
		ping_intensity.at<float>(i) = max_loc.x;
	}
	ping_intensity /= hist.cols;

	// convert to SonarBeam::beam data format (8-bit)
	std::cout << "Simulating beam data..." << std::endl;
	ping_intensity *= 255;
	ping_intensity.convertTo(data, CV_8U);

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
//	sonar.time_beams = NULL;
	sonar.number_of_beams = 1;
	sonar.number_of_bins = _number_of_bins;
//	sonar.start_bearing = NULL;
//	sonar.angular_resolution = NULL;
	sonar.sampling_interval = ((640.0 * ad_interval) * 1e-9);
	sonar.beamwidth_horizontal =  base::Angle::fromDeg(3.0);
	sonar.beamwidth_vertical = base::Angle::fromDeg(35.0);
	sonar.speed_of_sound = _speed_of_sound;

	return sonar;
}

}
