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
	// It will be needed to optimize this part yet
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


// The sonar receiver has an dynamic range (0..80dB = 0..255). Two parameters are used by
// Sonar in 'mtHeadCommand': ad_low, which sets the Lower boundary of the sampling window,
// it can be increased to make the Sonar display less sensitive and filter out background
// and receiver self noise; and ad_span sets the width of the sampling window and therefore
// acts as a Contrast control.
std::vector<uint8_t> ScanSonar::applyDynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span){

	for(uint i=0; i<data.size(); i++)
	{
		if (data[i] <= ad_low)
			data[i] = 0;
		else if (data[i] >= ad_low + ad_span)
			data[i] = 255;
		else
			data[i] = (uint8_t) ((double)(data[i]-ad_low)*255.0/(double)ad_span);
	}
	return data;
}

// Simulate a base::samples::SonarBeam data and update the sonar head position
base::samples::SonarBeam ScanSonar::simulateSonarBeam (std::vector<uint8_t> data, float step_angle)
{
	base::samples::SonarBeam beam;

	beam.bearing = base::Angle::fromDeg(_bearing);
	beam.beam = data;

	_bearing += step_angle;

	return beam;

}

}
