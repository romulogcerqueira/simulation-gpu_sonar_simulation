#ifndef GPU_SONAR_SIMULATION_MULTIBEAM_SONAR_HPP
#define GPU_SONAR_SIMULATION_MULTIBEAM_SONAR_HPP

#include "CommonSonar.hpp"

namespace gpu_sonar_simulation {
class MultibeamSonar : public CommonSonar {

public:

	MultibeamSonar():
		CommonSonar(),
		_beam_count(256)
	{
	    _beam_width = base::Angle::fromDeg(120.0);
	    _beam_height = base::Angle::fromDeg(20.0);
	};

	base::samples::Sonar simulateMultiBeam(const std::vector<float>& data);
	std::vector<float> codeSonarData(const cv::Mat3f& cv_image);

	uint32_t getBeamCount() const {
	    return _beam_count;
	}

	void setBeamCount(uint32_t beamCount) {
	    _beam_count = beamCount;
	}

private:
	uint32_t _beam_count;
};

} // end namespace gpu_sonar_simulation

#endif
