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
	sonar_scan.memory_layout_column = false;
	sonar_scan.polar_coordinates = true;

	return sonar_scan;
}

// Split image in beam parts. The shader is not radially spaced equally
// over the FOV-X degree sector, so it is necessary to identify which column
// is contained on each beam.
std::vector<uint8_t> MultibeamSonar::codeSonarData(cv::Mat3f cv_image) {

	std::vector<cv::Mat> shader;
	cv::split(cv_image, shader);

	// associates shader columns with their respective beam
	std::vector<uint8_t> sonar_data(_number_of_beams * _number_of_bins, 0);

	float interval = 1.0 / (_number_of_beams * 0.5);
	int middle_img = shader[0].cols * 0.5;
	double a = _number_of_beams * 1.0 / 2;

	for (int i = 0; i < shader[0].cols; i++) {

		// Checks normal values in each column
		if (cv::countNonZero(shader[0].col(i))) {

			int col_start = i;
			int col_end = -1;
			int id_beam;

			// gets the maximum angle in that column
			double max, max2;
			cv::minMaxIdx(shader[2].col(i), NULL, &max);

			i < middle_img ? max = -max : max += 0;
			id_beam = a * (max + 1);

			while (col_end == -1) {
				cv::minMaxIdx(shader[2].col(++i), NULL, &max2);
				i < middle_img ? max2 = -max2 : max2 += 0;
				int id_curr = a * (max2 + 1);
				if (id_curr != id_beam)
					col_end = --i;
			}

			// gets the ROI (beam) of shader image
			cv::Mat cv_roi = cv_image.colRange(col_start, col_end + 1);

			// processes shader informations
			cv::Mat raw_intensity = decodeShaderImage(cv_roi);
			std::vector<uint8_t> roi_data = getPingData(raw_intensity);
			for (int i = 0; i < _number_of_bins; ++i)
				sonar_data[_number_of_bins * id_beam + i] = roi_data[i];
		}
	}

	return sonar_data;
}

// Receives sonar data and plot it using polar coordinates
// TODO: Improve drawing to fill missing spaces between sonar data using a pixel interpolation technique.
void MultibeamSonar::plotSonarData(base::samples::SonarScan sonar, float range, int gain) {

	// check if number_of_bins was changed
	int nbins = sonar.number_of_bins;
	_viewer = cv::Mat::zeros(nbins * 1.2, nbins * 2, CV_8UC3);

	// display parameters
	double fovx = sonar.beamwidth_horizontal.rad;
	double step_angle = fovx / sonar.number_of_beams;
	double start_angle = sonar.start_bearing.rad - M_PI_2;
	double end_angle = start_angle + fovx;
	cv::Point center(_viewer.cols / 2, _viewer.rows * 0.9);

	// plot sonar data
	cv::Mat1b tempPlot = cv::Mat1b::zeros(_viewer.size());
	for (int i = 0; i < sonar.number_of_beams; ++i) {
		double bearing = start_angle + step_angle * i;
		double c = cos(bearing);
		double s = sin(bearing);

		for (int j = 0; j < nbins; ++j)
			tempPlot[(int) (center.y + s * j)][(int) (center.x + c * j)] = sonar.data[nbins * i + j];
	}

	cv::applyColorMap(tempPlot, _viewer, COLORMAP_HOT);

	// apply gain
	float g = gain * 1.0 / 50.0;
	_viewer *= g;

	// plot sonar grid
	cv::Scalar color(153, 255, 204);
	cv::line(_viewer, center, cv::Point(center.x + cos(start_angle) * nbins, center.y + sin(start_angle) * nbins), color, 1, CV_AA);
	cv::line(_viewer, center, cv::Point(center.x + cos(end_angle) * nbins, center.y + sin(end_angle) * nbins), color, 1, CV_AA);
	cv::line(_viewer, center, cv::Point(center.x, center.y - nbins), color, 1, CV_AA);

	float label = 0.0;
	for (int i = nbins / 5; i <= nbins; i += nbins / 5) {
		label += range / 5;
		stringstream ss;
		ss << label;
		cv::ellipse(_viewer, center, cv::Size(i, i), 180, base::Angle::rad2Deg(start_angle) + 180, base::Angle::rad2Deg(end_angle) + 180, color, 1, CV_AA);
		cv::putText(_viewer, ss.str(), cv::Point(center.x + cos(end_angle) * i + 10, center.y + sin(end_angle) * i + 10), FONT_HERSHEY_SIMPLEX, 0.4, color, 1, CV_AA);
	}
}
}
