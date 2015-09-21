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
std::vector<uint8_t> applyDynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span) {

	for (uint i = 0; i < data.size(); i++) {
		if (data[i] <= ad_low)
			data[i] = 0;
		else if (data[i] >= ad_low + ad_span)
			data[i] = 255;
		else
			data[i] = (uint8_t) ((double) (data[i] - ad_low) * 255.0 / (double) ad_span);
	}
	return data;
}

// Plot the accumulated intensity as an histogram
cv::Mat plotNormalHistogram(cv::Mat raw_intensity, int bins) {

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
cv::Mat plotDepthHistogram(cv::Mat raw_image, int bins) {

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

// Receives sonar data and plot it using polar coordinates
// TODO: Improve drawing to fill missing spaces between sonar data using a pixel interpolation technique.
cv::Mat plotSonarData(base::samples::SonarScan sonar, float range, float gain) {

	// check if number_of_bins was changed
	int nbins = sonar.number_of_bins;
	cv::Mat viewer = cv::Mat::zeros(nbins * 1.2, nbins * 2, CV_8UC3);

	// display parameters
	double fovx = sonar.beamwidth_horizontal.rad;
	double step_angle = fovx / sonar.number_of_beams;
	double start_angle = sonar.start_bearing.rad - M_PI_2;
	double end_angle = start_angle + fovx;
	cv::Point center(viewer.cols / 2, viewer.rows * 0.9);

	// plot sonar data
	cv::Mat1b tempPlot = cv::Mat1b::zeros(viewer.size());
	for (int i = 0; i < sonar.number_of_beams; ++i) {
		double bearing = start_angle + step_angle * i;
		double c = cos(bearing);
		double s = sin(bearing);

		for (int j = 0; j < nbins; ++j)
			tempPlot[(int) (center.y + s * j)][(int) (center.x + c * j)] = sonar.data[nbins * i + j];
	}

	cv::applyColorMap(tempPlot, viewer, COLORMAP_HOT);

	// apply gain
	float g = gain / 0.5;
	viewer *= g;

	// plot sonar grid
	cv::Scalar color(153, 255, 204);
	cv::line(viewer, center, cv::Point(center.x + cos(start_angle) * nbins, center.y + sin(start_angle) * nbins), color, 1, CV_AA);
	cv::line(viewer, center, cv::Point(center.x + cos(end_angle) * nbins, center.y + sin(end_angle) * nbins), color, 1, CV_AA);
	cv::line(viewer, center, cv::Point(center.x, center.y - nbins), color, 1, CV_AA);

	float label = 0.0;
	for (int i = nbins / 5; i <= nbins; i += nbins / 5) {
		label += range / 5;
		std::stringstream ss;
		ss << label;
		cv::ellipse(viewer, center, cv::Size(i, i), 180, base::Angle::rad2Deg(start_angle) + 180, base::Angle::rad2Deg(end_angle) + 180, color, 1, CV_AA);
		cv::putText(viewer, ss.str(), cv::Point(center.x + cos(end_angle) * i + 10, center.y + sin(end_angle) * i + 10), FONT_HERSHEY_SIMPLEX, 0.4, color, 1, CV_AA);
	}

	return viewer;
}

// Receives sonar data and plot it using polar coordinates
cv::Mat plotSonarData(base::samples::SonarBeam sonar, float range, float gain, cv::Mat cv_sonar, double step) {

	int bins = sonar.beam.size();

	if ((cv_sonar.size().width - 10) != bins * 2)
		cv_sonar = cv::Mat(bins * 2 + 10, bins * 2 + 10, CV_8UC3);

	cv::Point center(cv_sonar.cols / 2, cv_sonar.rows / 2);

	double s, c;

	for (double k = 0.0; k < step; k += step / 4) {
		double angle = sonar.bearing.rad + k;

		// plot sonar data
		s = sin(-angle);
		c = cos(-angle);

		for (int i = 0; i < bins; ++i)
			cv::circle(cv_sonar, cv::Point(center.x + c * i, center.y + s * i), 1, CV_RGB(sonar.beam[i], sonar.beam[i], sonar.beam[i]));
	}

	cv::Mat viewer;
	cv_sonar.copyTo(viewer);
	cv::applyColorMap(viewer, viewer, COLORMAP_HOT);

	// apply gain
	float g = gain / 0.5;
	viewer *= g;

	// plot grid
	cv::Scalar color(153,255,204);
	cv::line(viewer, cv::Point(center.x, 5), cv::Point(center.x, bins*2+5), color, 1, CV_AA);
	cv::line(viewer, cv::Point(5, center.y), cv::Point(bins*2+5, center.y), color, 1, CV_AA);
	float label = 0;
	for (int i = bins / 5; i <= bins; i += bins / 5) {
		cv::circle(viewer, center, i - 1, color, 1, CV_AA);
		label += range / 5;
		std::stringstream ss;
		ss << label;
		cv::putText(viewer, ss.str(), cv::Point(center.x + i - 20, center.y), FONT_HERSHEY_SIMPLEX, 0.4, color, 1, CV_AA);
	}
	cv::line(viewer, center, center + cv::Point(c * bins, s * bins), color, 1, CV_AA);

	return viewer;
}

}

