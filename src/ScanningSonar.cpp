#include "ScanningSonar.hpp"
#include <iostream>

using namespace std;
using namespace cv;
using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {


// Configure the ScanningSonar
void ScanningSonar::setup(SonarConfig config)
{
    int ad_interval = (((config.resolution/config.speed_of_sound)*2.0)*1e9)/640.0;
    int number_of_bins = config.max_distance/config.resolution;
    uint16_t left_limit = (((M_PI-config.left_limit.rad)/(M_PI*2.0))*6399.0);
    uint16_t right_limit = (((M_PI-config.right_limit.rad)/(M_PI*2.0))*6399.0);
    uint8_t motor_step_angle_size = config.angular_resolution.rad/(M_PI*2.0)*6399.0;

    std::cout << "Configure Sonar:" << std::endl;
	std::cout << "ad_interval:" << ad_interval << " number_of_bins:" << number_of_bins << " left_limit:" << left_limit
				<< " right_limit:" << right_limit << " motor_step_angle_size:" << (int)motor_step_angle_size
				<< " ad_low:" << (int)config.ad_low << " ad_span:" << (int)config.ad_span
				<< std::endl;

	 //check configuration
	if(config.angular_resolution.rad < 0 || config.angular_resolution.rad / (0.05625/180.0*M_PI) > 255.0 ||
	   ad_interval < 0 || ad_interval > 1500 || number_of_bins < 0 || number_of_bins > 1500 )
	{
		if(config.angular_resolution.rad < 0 || config.angular_resolution.rad / (0.05625/180.0*M_PI) > 255.0){
			throw std::runtime_error("ScanningSonar:: Invalid angular rad configuration.");
		}
		if(ad_interval < 0 || ad_interval > 1500){
			throw std::runtime_error("ScanningSonar:: Invalid ad_interval configuration.");
		}
		if(number_of_bins < 0 || number_of_bins > 1500){
			throw std::runtime_error("ScanningSonar:: Invalid numbers of bins.");
		}
		throw std::runtime_error("ScanningSonar:: invalid configuration.");
	}

	_speed_of_sound = config.speed_of_sound;
	_number_of_bins = number_of_bins;
	_ad_interval = ad_interval;
	_motor_step_angle_size = motor_step_angle_size;
	_config = config;
}


// Receive the depth and normal matrixes from the source image and
// return the maximum intensity for each bin
cv::Mat ScanningSonar::decodeRawImage(cv::Mat raw_image) {

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

		}
	}

	// calculate the mean of each intensity bin
	for (int i = 0; i < _number_of_bins; i++)
		raw_intensity.at<float>(i) /= (float) bin_depth.at<int>(i);

	return raw_intensity;
}


// Calculate the maximum intensity of one ping (8-bit format) by normal histogram
std::vector<uint8_t> ScanningSonar::getPingIntensity(cv::Mat raw_intensity) {

	std::vector<uint8_t> data;

	// convert to SonarScan::data format (8-bit)
	raw_intensity.convertTo(data, CV_8U, 255);

	// apply dynamic range control
	return dynamicRangeControl(data,_config.ad_low, _config.ad_span);
}


//	The sonar receiver has an 80dB dynamic range, and signal levels are processed internally by the sonar head such
//	that 0..80dB = 0..255. Two parameters are sent to Sonar in 'mtHeadCommand': ad_low, which sets the Lower boundary of
//	the sampling window, it can be increased to	make the Sonar display less sensitive and filter out background and receiver self noise;
//	and ad_span sets the width of the sampling window and therefore acts as a Contrast control.
std::vector<uint8_t> ScanningSonar::dynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span){

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

// Accepts the input value x then returns it's sigmoid value in float
double ScanningSonar::sigmoid(double x) {

	double l = 1, k = 30, x0 = 0.7777;
	double exp_value, return_value;

	exp_value = exp( -k * (x - x0));
	return_value = l / (1 + exp_value);

	return return_value;
}

// Accepts the input value x then returns it's half gaussian value in float
double ScanningSonar::halfGaussian(double x) {

	double a = 1.0, b = 1.0, c = 5.0;
	double exp_value, return_value;

	exp_value = exp(-(pow(x - b, 2) / 2 * pow(c, 2)));

	return_value = a * exp_value;

	return return_value;
}


// Generate simulated SonarBeam data
base::samples::SonarBeam ScanningSonar::simulateSonarBeam(std::vector<uint8_t> data)
{
	base::samples::SonarBeam sonar_beam;

	// calculate bearing
	sonar_beam.time = base::Time::now();
	sonar_beam.sampling_interval = ((640.0*_ad_interval)*1e-9);
	sonar_beam.bearing = base::Angle::fromRad(-(_bearing/6399.0*2.0*M_PI)+M_PI);
	sonar_beam.speed_of_sound = _speed_of_sound;
	sonar_beam.beamwidth_horizontal =  3.0/180.0*M_PI;
	sonar_beam.beamwidth_vertical =  35.0/180.0*M_PI;
	sonar_beam.beam = data;

	// setting bearing
	_bearing = _bearing + _motor_step_angle_size;
	if(_bearing > 6400) _bearing -= 6400;

	return sonar_beam;
}
}
