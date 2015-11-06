#include "ScanSonar.hpp"
#include <iostream>

using namespace gpu_sonar_simulation;
using namespace std;

namespace gpu_sonar_simulation {

// Simulate a base::samples::SonarBeam data and update the sonar head position
base::samples::SonarBeam ScanSonar::simulateSonarBeam (const std::vector<uint8_t>& data) {
	base::samples::SonarBeam beam;

	beam.time = base::Time::now();
	beam.bearing = base::Angle::fromRad(-_bearing + M_PI_2);
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
}
