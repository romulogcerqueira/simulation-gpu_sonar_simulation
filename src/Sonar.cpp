#include "Sonar.hpp"

using namespace gpu_sonar_simulation;
using namespace cv;

void Sonar::decodeShader(const cv::Mat& cv_image, std::vector<float>& bins) {
    bins.assign(beam_count * bin_count, 0.0);

    double const beam_size = beam_width.getRad() / beam_count;
    double const half_fovx = beam_width.getRad() / 2;
    double const half_width = static_cast<double>(cv_image.cols) / 2;
    double const angle2x = half_width / tan(half_fovx);

    // associates shader columns with their respective beam
    for (unsigned int beam_idx = 0; beam_idx < beam_count; ++beam_idx) {
        int min_col = round(half_width + tan(-half_fovx + beam_idx * beam_size) * angle2x);
        int max_col = round(half_width + tan(-half_fovx + (beam_idx + 1) * beam_size) * angle2x);
        cv::Mat cv_roi = cv_image.colRange(min_col, max_col);

        std::vector<float> raw_intensity;
        convertShader(cv_roi, raw_intensity);
        for (unsigned int bin_idx = 0; bin_idx < bin_count; bin_idx++)
            bins[bin_count * beam_idx + bin_idx] = raw_intensity[bin_idx];
    }
}

// Simulate one sonar reading in the Rock's structure
base::samples::Sonar Sonar::simulateSonar(const std::vector<float>& bins, float range) {
    base::samples::Sonar sonar;
    sonar.time = base::Time::now();
    sonar.bin_duration = base::Time::fromSeconds(getSamplingInterval(range) / 2.0);
    sonar.beam_width = beam_width;
    sonar.beam_height = beam_height;
    sonar.speed_of_sound = speed_of_sound;
    sonar.bin_count = bin_count;
    sonar.beam_count = beam_count;
    sonar.bins = bins;

    return sonar;
}

void Sonar::convertShader(const cv::Mat& cv_image, std::vector<float>& bins) {
    if (cv_image.type() != CV_32FC3)
        throw std::invalid_argument("Invalid shader image format.");

    int default_bins = 256;
    std::vector<int> bins_depth(default_bins, 0);
    std::vector<float> bins_normal(default_bins, 0.0);

    // calculate depth histogram
    for (cv::MatConstIterator_<Vec3f> px = cv_image.begin<Vec3f>(); px != cv_image.end<Vec3f>(); ++px) {
        int bin_idx = (*px)[1] * (default_bins - 1);
        bins_depth[bin_idx]++;
    }

    // calculate bins intesity using normal values, depth histogram and sigmoid function
    for (cv::MatConstIterator_<Vec3f> px = cv_image.begin<Vec3f>(); px != cv_image.end<Vec3f>(); ++px) {
        int bin_idx = (*px)[1] * (default_bins - 1);
        float intensity = (1.0 / bins_depth[bin_idx]) * sigmoid((*px)[0]);
        bins_normal[bin_idx] += intensity;
    }

    // rescale the bins intensity using a linear transformation
    rescaleIntensity(bins_normal, bins);
}

void Sonar::rescaleIntensity(const std::vector<float>& src, std::vector<float>& dst) {
    double rate = bin_count * 1.0 / src.size();
    dst.assign(bin_count, 0.0);

    // Rescale the accumulated normal vector to the number of bins desired
    for (unsigned int idx = 0; idx < src.size() - 1; ++idx) {
        double new_idx = idx * rate;
        dst[new_idx] = src[idx];

        if (src[idx + 1]) {
            double next_idx = (idx + 1) * rate;
            double local_slope = (src[idx + 1] - src[idx]) / (next_idx - new_idx);
            double local_const = src[idx] - local_slope * new_idx;

            for (double j = new_idx + 1; j < next_idx; j += 1.0)
                dst[j] = local_slope * j + local_const;
        }
    }
}

float Sonar::sigmoid(float x) {
	float l = 1, k = 18, x0 = 0.666666667;
	float exp_value;

	// Exponential calculation
	exp_value = exp((double) (-k * (x - x0)));

	// Final sigmoid value
	return (l / (1 + exp_value));
}

float Sonar::getSamplingInterval(float range) {
	float travel_time = range * 2.0 / speed_of_sound;
	return travel_time / bin_count;
}

void Sonar::applyAdditionalGain(std::vector<float>& bins, float gain) {
    float gain_factor = 1 + gain / 0.25;
    std::transform(bins.begin(), bins.end(), bins.begin(), std::bind1st(std::multiplies<float>(), gain_factor));
    std::replace_if(bins.begin(), bins.end(), std::bind2nd(std::greater<float>(), 1.0), 1.0);
}
