#include "SonarSim.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

// Calculate the maximum intensity of each bin
cv::Mat SonarSim::getPingIntensity(cv::Mat hist) {

	cv::Mat ping_intensity = cv::Mat::zeros(hist.rows, 1, CV_32F);
	std::vector<int> vec;
	int id_max_value;

	for (int i = 0; i < hist.rows; i++) {
		hist.row(i).copyTo(vec);
		id_max_value = std::distance(vec.begin(),
				std::max_element(vec.begin(), vec.end()));
		ping_intensity.at<float>(i) = (float) id_max_value
				/ (float) hist.cols;
	}

	return ping_intensity;
}

// Receive the depth and normal matrixes from the source image and
// return the maximum intensity for each bin
cv::Mat SonarSim::decodeRawImage(cv::Mat raw_image, int num_bins, int slices =
		10) {

	std::cout << "Decoding simulated image..." << std::endl;

	if (raw_image.type() != CV_32FC3) {
		std::cout << "Invalid image format!" << std::endl;
		exit(0);
	}

	float interval = 1.0 / (float) num_bins;

	std::vector<int> bin_depth(num_bins, 0);
	std::vector<std::vector<float> > bin_normal(num_bins);

	cv::Mat histogram = cv::Mat::zeros(num_bins, slices, CV_32S);
	cv::Mat ping_intensity;

	// organize depth and normal values
	int ind_bin, ind_hist;
	for (int i = 0; i < raw_image.rows; i++) {
		for (int j = 0; j < raw_image.cols; j++) {
			ind_bin = raw_image.at<Vec3f>(i, j)[0] / interval;
			if (ind_bin == num_bins)
				ind_bin--;
			bin_depth[ind_bin]++;
			bin_normal[ind_bin].push_back(raw_image.at<Vec3f>(i, j)[1]);
		}
	}

	// calculate the normal histogram for each bin
	for (int i = 0; i < num_bins; i++) {
		for (uint j = 0; j < bin_normal[i].size(); j++) {
			ind_hist = (int) (bin_normal[i][j] * slices);
			if (ind_hist == slices)
				ind_hist--;
			histogram.at<int>(i, ind_hist)++;}
		}

	ping_intensity = getPingIntensity(histogram);

	return ping_intensity;
}
}
