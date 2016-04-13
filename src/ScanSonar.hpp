#ifndef GPU_SONAR_SIMULATION_SCAN_SONAR_HPP
#define GPU_SONAR_SIMULATION_SCAN_SONAR_HPP

#include "CommonSonar.hpp"

namespace gpu_sonar_simulation {
class ScanSonar : public CommonSonar {

public:

	ScanSonar():
		CommonSonar(),
		_ping_pong_mode(false),
		_reverse_scan(false)
	{
	    _beam_width = base::Angle::fromDeg(3.0);
	    _beam_height = base::Angle::fromDeg(35.0);
	    _bearing = base::Angle::fromRad(0.0);
	    _start_angle = base::Angle::fromRad(-M_PI);
	    _end_angle = base::Angle::fromRad(M_PI);
	    _step_angle = base::Angle::fromDeg(0.9);
	};

	base::samples::Sonar simulateSingleBeam(const std::vector<float>& data);
	void moveHeadPosition();

	bool isReverseScan() const {
		return _reverse_scan;
	}

	void setReverseScan(bool reverseScan) {
		_reverse_scan = reverseScan;
	}

	bool isPingPongMode() const {
		return _ping_pong_mode;
	}

	void setPingPongMode(bool pingPongMode) {
		_ping_pong_mode = pingPongMode;
	}

    const base::Angle& getBearing() const {
        return _bearing;
    }

    void setBearing(const base::Angle& bearing) {
        _bearing = bearing;
    }

    const base::Angle& getEndAngle() const {
        return _end_angle;
    }

    void setEndAngle(const base::Angle& endAngle) {
        _end_angle = endAngle;
    }

    const base::Angle& getStartAngle() const {
        return _start_angle;
    }

    void setStartAngle(const base::Angle& startAngle) {
        _start_angle = startAngle;
    }

    const base::Angle& getStepAngle() const {
        return _step_angle;
    }

    void setStepAngle(const base::Angle& stepAngle) {
        _step_angle = stepAngle;
    }

private:
	base::Angle _bearing;
	base::Angle _start_angle;
	base::Angle _end_angle;
	base::Angle _step_angle;

	bool _ping_pong_mode;
	bool _reverse_scan;
};

} // end namespace gpu_sonar_simulation

#endif
