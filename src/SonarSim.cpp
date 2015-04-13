#include "SonarSim.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

// Calculate the maximum intensity of each bin
cv::Mat SonarSim::getPingIntensity(cv::Mat hist) {

	cv::Mat ping_intensity = cv::Mat::zeros(hist.rows, 1, CV_32F);

	for (int i = 0; i < hist.rows; i++) {
		cv::Point max_loc;
		cv::minMaxLoc(hist.row(i), NULL, NULL, NULL, &max_loc);
		ping_intensity.at<float>(i) = max_loc.x;
	}
	ping_intensity /= hist.cols;

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

	// organize depth and normal values and generate the histogram
	for (int i = 0; i < raw_image.rows; i++) {
		for (int j = 0; j < raw_image.cols; j++) {
			int id_bin = raw_image.at<Vec3f>(i, j)[0] / interval;
			if (id_bin == num_bins)
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

}
