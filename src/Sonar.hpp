#ifndef GPU_SONAR_SIMULATION_SONAR_HPP
#define GPU_SONAR_SIMULATION_SONAR_HPP

// C++ includes
#include <vector>

// Rock includes
#include <base/samples/Sonar.hpp>
#include <base/Angle.hpp>

// Opencv includes
#include <opencv2/core/core.hpp>

namespace gpu_sonar_simulation {

class Sonar {

public:
	/** Number of bins in a beam */
	uint32_t bin_count;

	/** Number of beams in the structure */
	uint32_t beam_count;

	/** Opening of the beam orhogonal to the device's Z direction */
	base::Angle beam_width;

	/** Opening of the beam along the device's Z direction */
	base::Angle beam_height;

	/** The speed of sound in the water in m/s */
	float speed_of_sound;

	Sonar()
		: bin_count(500)
		, beam_count(0)
		, beam_width(base::Angle::fromRad(0.0))
		, beam_height(base::Angle::fromRad(0.0))
		, speed_of_sound(base::samples::Sonar::getSpeedOfSoundInWater())
	{}

	Sonar(uint32_t bin_count, uint32_t beam_count, base::Angle beam_width, base::Angle beam_height)
		: bin_count(bin_count)
		, beam_count(beam_count)
		, beam_width(beam_width)
		, beam_height(beam_height)
		, speed_of_sound(base::samples::Sonar::getSpeedOfSoundInWater())
	{}

	/**
	 *  Split the shader image in beam parts. The shader is not radially spaced equally
	 *  over the FOV-X degree sector, so it is needed to identify which column is contained
	 *  on each beam.
	 *  @param cv_image: the shader image (normal, depth and angle informations) in float
	 *  @param bins: the output simulated sonar data (all beams) in float
	 */
	void decodeShader(const cv::Mat& cv_image, std::vector<float>& bins);

	/**
	 *  Encapsulate the simulated sonar data in the Rock's sonar datatype.
	 *  @param bins: the simulated sonar data in float
	 *  @param sonar: the simulated sonar in the Rock's structure
	 *  @param bearings: the set of bearings
	 *  @param range: the maximum coveraged area in meters
	 */
	void simulateSonar(const std::vector<float>& bins, base::samples::Sonar& sonar, const std::vector<base::Angle>& bearings, float range);

	/**
	 *  Apply an additional gain in the simulated sonar data.
	 *  @param bins: the simulated sonar data in float
	 *  @param gain: the additional gain percent (0.0 - 1.0)
	 */
	void applyAdditionalGain(std::vector<float>& bins, float gain);

private:
	/**
	 *  Convert the shader image (normal and depth) in bins intensity (one beam).
	 *  @param cv_image: the shader image (normal and depth informations) in float
	 *  @param bins: the output simulated sonar data (one beam) in float
	 */
	void convertShader(const cv::Mat& cv_image, std::vector<float>& bins);

	/**
	*  The shader has a precision float limitation (1/256 = 0.00390625). To avoid "black
	"  wholes" in the final sonar image if number of bins be more than 256, this function
	*  rescale the sonar intensity data applying a linear transformation.
	*  @param src: the raw bins intensity values in float
	*  @param dst: the rescaled bins intensity values in float
	*/
	void rescaleIntensity(const std::vector<float>& src, std::vector<float>& dst);

	/**
	 *  Accept the input value x then returns it's sigmoid value in float.
	 *  @param x: the input value in float
	 *  @return the sigmoid value in float
	 */
	float sigmoid(float x);

	/**
	 *  Calculate the sample time period that is applied to the received sonar echo signal.
	 *  @param range: the range (meters) in float
	 *  @return the sampling interval
	 */
	float getSamplingInterval(float range);
};
} // end namespace gpu_sonar_simulation

#endif
