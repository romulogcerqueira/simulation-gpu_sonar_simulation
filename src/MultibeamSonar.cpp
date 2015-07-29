#include "MultibeamSonar.hpp"
#include <iostream>

using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

// Simulate a base::samples::SonarBeam data and update the sonar head position
base::samples::SonarScan MultibeamSonar::simulateSonarScan(std::vector<uint8_t> data) {

	base::samples::SonarScan sonar_scan;

	sonar_scan.time = base::Time::now();
	sonar_scan.data = data;
	sonar_scan.number_of_beams = _number_of_beams;
	sonar_scan.number_of_bins = _number_of_bins;
	sonar_scan.start_bearing = base::Angle::fromDeg(_start_bearing);
	sonar_scan.angular_resolution = base::Angle::fromDeg(_angular_resolution);
	sonar_scan.sampling_interval = getSamplingInterval();
	sonar_scan.speed_of_sound = _speed_of_sound;
	sonar_scan.beamwidth_horizontal = base::Angle::fromDeg(_beamwidth_horizontal);
	sonar_scan.beamwidth_vertical = base::Angle::fromDeg(_beamwidth_vertical);
	sonar_scan.memory_layout_column = true;
	sonar_scan.polar_coordinates = true;

	return sonar_scan;
}

// Split shader image in beam parts
std::vector<cv::Mat> MultibeamSonar::splitShaderImage(cv::Mat cv_image) {

	std::vector<cv::Mat> output;

	for (int i = 0; i < cv_image.cols; i += _pixels_per_beam) {
		cv::Mat current(cv_image, cv::Rect(i, 0, _pixels_per_beam, cv_image.rows));
		output.push_back(current);
	}

	return output;
}
}
