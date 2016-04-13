#ifndef GPU_SONAR_SIMULATION_COMMON_SONAR_HPP
#define GPU_SONAR_SIMULATION_COMMON_SONAR_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <base/samples/Sonar.hpp>
#include <base/Angle.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class CommonSonar {

public:

	CommonSonar():
		_number_of_bins(500),
		_gain(0.5f),
		_speed_of_sound(1500.0f),
		_range(50.0f)
	{
	    _beam_width = base::Angle::fromDeg(0.0);
	    _beam_height = base::Angle::fromDeg(0.0);
	};

	std::vector<float> decodeShaderImage(const cv::Mat& raw_image);
	double getSamplingInterval();

	int getNumberOfBins() const {
		return _number_of_bins;
	}

	void setNumberOfBins(int numberOfBins) {
		_number_of_bins = numberOfBins;
	}

	float getRange() const {
		return _range;
	}

	void setRange(float range) {
		_range = range;
	}

	float getSpeedOfSound() const {
		return _speed_of_sound;
	}

	void setSpeedOfSound(float speedOfSound) {
		_speed_of_sound = speedOfSound;
	}

    float getGain() const {
        return _gain;
    }

    void setGain(float gain) {
        _gain = gain;
    }

    const base::Angle& getBeamHeight() const {
        return _beam_height;
    }

    void setBeamHeight(const base::Angle& beamHeight) {
        _beam_height = beamHeight;
    }

    const base::Angle& getBeamWidth() const {
        return _beam_width;
    }

    void setBeamWidth(const base::Angle& beamWidth) {
        _beam_width = beamWidth;
    }

private:
    float sigmoid(float value);
    std::vector<float> rescaleIntensity(const std::vector<float>& bins_normal);

protected:
	int _number_of_bins;
	float _gain;
	float _speed_of_sound;
	float _range;
	base::Angle _beam_width;
	base::Angle _beam_height;
};

} // end namespace gpu_sonar_simulation

#endif
