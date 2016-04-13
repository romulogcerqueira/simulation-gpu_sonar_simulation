#ifndef GPU_SONAR_SIMULATION_MULTIBEAM_SONAR_HPP
#define GPU_SONAR_SIMULATION_MULTIBEAM_SONAR_HPP

#include "CommonSonar.hpp"

namespace gpu_sonar_simulation {
class MultibeamSonar : public CommonSonar {

public:

	MultibeamSonar():
		CommonSonar(),
		_number_of_beams(256),
		_pixels_per_beam(2)
	{
	    _beam_width = base::Angle::fromDeg(120.0);
	    _beam_height = base::Angle::fromDeg(20.0);
	};

	base::samples::Sonar simulateMultibeamSonar(const std::vector<float>& data);
	std::vector<uint8_t> codeSonarData(const cv::Mat3f& cv_image);

	unsigned int getNumberOfBeams() const {
		return _number_of_beams;
	}

	void setNumberOfBeams(unsigned int numberOfBeams) {
		_number_of_beams = numberOfBeams;
	}

	unsigned int getPixelsPerBeam() const {
		return _pixels_per_beam;
	}

	void setPixelsPerBeam(unsigned int pixelsPerBeam) {
		_pixels_per_beam = pixelsPerBeam;
	}

private:
	unsigned int _number_of_beams;
	unsigned int _pixels_per_beam;
};

} // end namespace gpu_sonar_simulation

#endif
