#include "ScanSonar.hpp"
#include <iostream>

using namespace gpu_sonar_simulation;
using namespace std;

namespace gpu_sonar_simulation {

// Simulate a base::samples::SonarBeam data and update the sonar head position
base::samples::SonarBeam ScanSonar::simulateSonarBeam (std::vector<uint8_t> data) {
	base::samples::SonarBeam beam;

	beam.time = base::Time::now();
	beam.bearing = base::Angle::fromDeg(-_bearing + 90.0f);
	beam.sampling_interval = getSamplingInterval();
	beam.speed_of_sound = _speed_of_sound;
	beam.beamwidth_horizontal = base::Angle::deg2Rad(_beamwidth_horizontal);
	beam.beamwidth_vertical = base::Angle::deg2Rad(_beamwidth_vertical);
	beam.beam = data;

	// if ping_pong is false, the sonar scans from _left_limit to _right_limit in loop
	if (!_ping_pong_mode) {
		_bearing += _step_angle;

		if (_bearing > _end_angle)
			_bearing = _start_angle;
	}

	// otherwise, the sonar scans from _left_limit to _right_limit and vice versa in loop
	else {
		// it scans from _left_limit to _right_limit
		if (!_reverse_scan) {
			_bearing += _step_angle;

			if (_bearing > _end_angle) {
				_bearing = _end_angle;
				_reverse_scan = true;
			}
		}

		// it scans from _right_limit to _left_limit
		else {
			_bearing -= _step_angle;

			if (_bearing < _start_angle) {
				_bearing = _start_angle;
				_reverse_scan = false;
			}
		}
	}

	return beam;
}

// Receives sonar data and plot it using polar coordinates
void ScanSonar::plotSonarData(base::samples::SonarBeam sonar, float range, int gain) {

	int bins = sonar.beam.size();

	if ((_output.size().width - 10) != bins * 2)
		_output = cv::Mat(bins * 2 + 10, bins * 2 + 10, CV_8UC3);

	cv::Point center(_output.cols / 2, _output.rows / 2);

	double s, c;

	for (double k = 0.0; k < _step_angle; k += _step_angle / 4) {
		double angle = sonar.bearing.rad + k * M_PI / 180.0;

		// plot sonar data
		s = sin(-angle);
		c = cos(-angle);

		for (int i = 0; i < bins; ++i)
			cv::circle(_output, cv::Point(center.x + c * i, center.y + s * i), 1, CV_RGB(sonar.beam[i], sonar.beam[i], sonar.beam[i]));
	}

	_output.copyTo(_viewer);
	cv::applyColorMap(_viewer, _viewer, COLORMAP_HOT);

	// apply gain
	float g = gain * 1.0 / 50.0;
	_viewer *= g;

	// plot grid
	cv::Scalar color(153,255,204);
	cv::line(_viewer, cv::Point(center.x, 5), cv::Point(center.x, bins*2+5), color, 1, CV_AA);
	cv::line(_viewer, cv::Point(5, center.y), cv::Point(bins*2+5, center.y), color, 1, CV_AA);
	float label = 0;
	for (int i = bins / 5; i <= bins; i += bins / 5) {
		cv::circle(_viewer, center, i - 1, color, 1, CV_AA);
		label += range / 5;
		stringstream ss;
		ss << label;
		cv::putText(_viewer, ss.str(), cv::Point(center.x + i - 20, center.y), FONT_HERSHEY_SIMPLEX, 0.4, color, 1, CV_AA);
	}
	cv::line(_viewer, center, center + cv::Point(c * bins, s * bins), color, 1, CV_AA);
}
}
