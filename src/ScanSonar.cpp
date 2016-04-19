#include "ScanSonar.hpp"
#include <iostream>

using namespace gpu_sonar_simulation;
using namespace std;

namespace gpu_sonar_simulation {

// Simulate a base::samples::Sonar data
base::samples::Sonar ScanSonar::simulateSingleBeam (const std::vector<float>& data) {
    base::samples::Sonar sonar;
    sonar.time = base::Time::now();
    sonar.bin_duration = base::Time::fromSeconds(getSamplingInterval() / 2.0);
    sonar.beam_width = _beam_width;
    sonar.beam_height = _beam_height;
    sonar.speed_of_sound = _speed_of_sound;
    sonar.bin_count = data.size();
    sonar.pushBeam(data, _bearing);

    return sonar;
}

// Update the scanning reader position
void ScanSonar::moveHeadPosition() {
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
}

}
