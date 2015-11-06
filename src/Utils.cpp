/*
 * Utils.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: romulogcerqueira
 */

#include "Utils.hpp"

using namespace cv;
using namespace osg;

namespace gpu_sonar_simulation {

// Convert the shader image from OSG to OpenCV
cv::Mat convertShaderOSG2CV(osg::ref_ptr<osg::Image> osg_image) {

	cv::Mat3f cvImage(osg_image->t(), osg_image->s());
	cvImage.data = osg_image->data();
	cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
	cv::flip(cvImage, cvImage, 0);

	return cvImage;
}

// The sonar receiver has an dynamic range (0..80dB = 0..255). Two parameters are used by
// Sonar in 'mtHeadCommand': ad_low, which sets the Lower boundary of the sampling window,
// it can be increased to make the Sonar display less sensitive and filter out background
// and receiver self noise; and ad_span sets the width of the sampling window and therefore
// acts as a Contrast control.
void applyDynamicRangeControl(std::vector<uint8_t>& data, uint8_t ad_low, uint8_t ad_span) {

	for (uint i = 0; i < data.size(); i++) {
		if (data[i] <= ad_low)
			data[i] = 0;
		else if (data[i] >= ad_low + ad_span)
			data[i] = 255;
		else
			data[i] = (uint8_t) ((double) (data[i] - ad_low) * 255.0 / (double) ad_span);
	}
}

// Plot the accumulated intensity as an histogram
cv::Mat plotNormalHistogram(const cv::Mat& raw_intensity, int bins) {

	double max;
	cv::minMaxIdx(raw_intensity, NULL, &max);

	cv::Mat1f new_bin = raw_intensity / max;
	cv::Mat cv_normal = cv::Mat3b::zeros(bins, bins);

	for (int i = 0; i < bins; i++) {
		cv::Scalar color;
		(i % 2) == 0 ? color = cv::Scalar(255, 255, 0) : color = cv::Scalar(0, 0, 255);
		cv::line(cv_normal, cv::Point(i, bins), cv::Point(i, bins * (1 - new_bin.at<float>(i))), color, 1);
	}

	return cv_normal;
}

// Plot the depth as an histogram
cv::Mat plotDepthHistogram(const cv::Mat& raw_image, int bins) {

	if (raw_image.type() != CV_32FC3) {
		std::cout << "Invalid shader image format!" << std::endl;
		exit(0);
	}

	// calculate depth histogram
	cv::Mat bins_depth = cv::Mat::zeros(bins, 1, CV_32S);
	std::vector<cv::Mat> shader;
	split(raw_image, shader);

	float range[] = { 0.0f, 1.0f };
	const float *hist_range = { range };

	cv::calcHist(&shader[1], 1, 0, cv::Mat(), bins_depth, 1, &bins, &hist_range);
	bins_depth.convertTo(bins_depth, CV_32S);

	double max;
	cv::minMaxIdx(bins_depth, NULL, &max);

	cv::Mat1f new_bin(bins_depth.size());
	new_bin = (bins_depth * 1.0) / (float) max;

	cv::Mat cv_depth = cv::Mat3b::zeros(bins, bins);

	for (int i = 0; i < bins; i++) {
		cv::Scalar color;
		(i % 2) == 0 ? color = cv::Scalar(255, 255, 0) : color = cv::Scalar(0, 0, 255);
		cv::line(cv_depth, cv::Point(i, bins), cv::Point(i, bins * (1 - new_bin.at<float>(i))), color, 1);
	}

	return cv_depth;
}
}

