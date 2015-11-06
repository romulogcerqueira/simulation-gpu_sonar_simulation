#include "CommonSonar.hpp"
#include <iostream>

using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

// Accepts the input value x then returns it's sigmoid value in float
float CommonSonar::sigmoid(float x) {

	float l = 1, k = 18, x0 = 0.666666667;
	float exp_value;

	// Exponential calculation
	exp_value = exp((double) (-k * (x - x0)));

	// Final sigmoid value
	return (l / (1 + exp_value));
}

// Receive shader image (normal and depth matrixes) and convert to get
// the bins intensities
std::vector<double> CommonSonar::decodeShaderImage(const cv::Mat& raw_image) {

    if (raw_image.type() != CV_32FC3)
        std::invalid_argument("Invalid shader image format!");

    // the shader has a precision float limitation (1/256 = 0.00390625). So if the
    // number of bins be more than 256, the depth histogram will present some
    // "black wholes" and this will reflect on final sonar image. To avoid this problem,
    // we need to rescale the sonar intensity data applying a linear transformation.
    int default_bins = 256;
    std::vector<int> bins_depth(default_bins, 0);
    std::vector<double> bins_normal(default_bins, 0);

    // calculate depth histogram
    for (cv::MatConstIterator_<Vec3f> it = raw_image.begin<Vec3f>(); it != raw_image.end<Vec3f>(); ++it)
        bins_depth[(*it)[1] * (default_bins - 1)]++;

    // calculate bins intensities using normal values, depth histogram and sigmoid function
    for (cv::MatConstIterator_<Vec3f> it = raw_image.begin<Vec3f>(); it != raw_image.end<Vec3f>(); ++it) {
        int id_bin = (*it)[1] * (default_bins - 1);
        bins_normal[id_bin] += (1.0 / bins_depth[id_bin]) * sigmoid((*it)[0]);
    }

    return rescaleIntensity(bins_normal);
}

// Rescale the accumulated normal vector to the number of bins desired
std::vector<double> CommonSonar::rescaleIntensity(const std::vector<double>& bins_normal) {

    double rate = _number_of_bins * 1.0 / bins_normal.size();
    std::vector<double> new_hist(_number_of_bins, 0);

    for (unsigned int i = 0; i < bins_normal.size() - 1; ++i) {
        double iNew = i * rate;
        new_hist[iNew] = bins_normal[i];

        if (bins_normal[i + 1] != 0) {
            double nextINew = (i + 1) * rate;
            double local_slope = (bins_normal[i + 1] - bins_normal[i]) / (nextINew - iNew);
            double local_const = bins_normal[i] - local_slope * iNew;
            for (double j = iNew + 1; j < nextINew; j += 1.0)
                new_hist[j] = local_slope * j + local_const;
        }
    }

    return new_hist;
}

// Calculate ping intensity in 8-bit format data
std::vector<uint8_t> CommonSonar::getPingData(std::vector<double>& raw_intensity) {

    std::transform(raw_intensity.begin(), raw_intensity.end(), raw_intensity.begin(), std::bind1st(std::multiplies<double>(), 255));
    std::vector<uint8_t> ping_intensity(raw_intensity.begin(), raw_intensity.end());

    return ping_intensity;
}

// Calculate the sample time period that is applied to the received Sonar echo signal
double CommonSonar::getSamplingInterval() {

	double travel_time = _range * 2.0 / _speed_of_sound;

	return travel_time / _number_of_bins;
}
}
