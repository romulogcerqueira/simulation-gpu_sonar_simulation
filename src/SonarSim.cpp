#include "SonarSim.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

SonarSim::SonarSim() {
	_num_bins = 50;
	_range = 20.0;
}

SonarSim::~SonarSim() {
}

// Receive the depth and normal matrixes from the source image and
// return the maximum intensity for each bin
std::vector<uint8_t> SonarSim::decodeRawImage(cv::Mat raw_image,
		int slices) {

	std::cout << "Decoding simulated image..." << std::endl;

	if (raw_image.type() != CV_32FC3) {
		std::cout << "Invalid image format!" << std::endl;
		exit(0);
	}

	float interval = 1.0 / (float) _num_bins;

	std::vector<int> bin_depth(_num_bins, 0);
	std::vector<std::vector<float> > bin_normal(_num_bins);

	cv::Mat histogram = cv::Mat::zeros(_num_bins, slices, CV_32S);

	// organize depth and normal values and generate the histogram
	for (int i = 0; i < raw_image.rows; i++) {
		for (int j = 0; j < raw_image.cols; j++) {
			int id_bin = raw_image.at<Vec3f>(i, j)[0] / interval;
			if (id_bin == _num_bins)
				id_bin--;
			bin_depth[id_bin]++;
			bin_normal[id_bin].push_back(raw_image.at<Vec3f>(i, j)[1]);
			int id_hist = (int) (raw_image.at<Vec3f>(i, j)[1] * slices);
			if (id_hist == slices)
				id_hist--;
			histogram.at<int>(id_bin, id_hist)++;
		}
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
	std::cout << "Simulating sonar data..." << std::endl;
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

}
