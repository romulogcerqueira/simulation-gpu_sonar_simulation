#include "Sonar.hpp"

using namespace gpu_sonar_simulation;
using namespace cv;

void Sonar::decodeShader(const cv::Mat& cv_image, std::vector<float>& bins) {
    bins.assign(beam_count * bin_count, 0.0);

    if (beam_cols.empty()) {
        double beam_size = beam_width.getRad() / beam_count;
        double half_fovx = beam_width.getRad() / 2;
        double half_width = static_cast<double>(cv_image.cols) / 2;
        double angle2x = half_width / tan(half_fovx);

        // associates shader columns with their respective beam
        for (unsigned int beam_idx = 0; beam_idx < beam_count; ++beam_idx) {
            int min_col = round(half_width + tan(-half_fovx + beam_idx * beam_size) * angle2x);
            int max_col = round(half_width + tan(-half_fovx + (beam_idx + 1) * beam_size) * angle2x);
            beam_cols.push_back(min_col);
            beam_cols.push_back(max_col);
        }
    }

    for (unsigned int beam_idx = 0; beam_idx < beam_count; ++beam_idx) {
        cv::Mat cv_roi = cv_image.colRange(beam_cols[beam_idx * 2], beam_cols[beam_idx * 2 + 1]);
        std::vector<float> raw_intensity;
        convertShader(cv_roi, raw_intensity);
        memcpy(&bins[bin_count * beam_idx], &raw_intensity[0], bin_count * sizeof(float));
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

    // calculate depth histogram
    std::vector<int> bins_depth(bin_count, 0);
    for (cv::MatConstIterator_<Vec3f> px = cv_image.begin<Vec3f>(); px != cv_image.end<Vec3f>(); ++px) {
        int bin_idx = (*px)[1] * (bin_count - 1);
        bins_depth[bin_idx]++;
    }

    // calculate bins intesity using normal values, depth histogram and sigmoid function
    bins.assign(bin_count, 0.0);
    for (cv::MatConstIterator_<Vec3f> px = cv_image.begin<Vec3f>(); px != cv_image.end<Vec3f>(); ++px) {
        if ((*px)[0]) {
            int bin_idx = (*px)[1] * (bin_count - 1);
            float intensity = (1.0 / bins_depth[bin_idx]) * sigmoid((*px)[0]);
            bins[bin_idx] += intensity;
        }
    }
}

float Sonar::sigmoid(float x) {
    float beta = 18, x0 = 0.666666667;
    float t = (x - x0) * beta;
    return (0.5 * tanh(0.5 * t) + 0.5);
}

float Sonar::getSamplingInterval(float range) {
    float travel_time = range * 2.0 / speed_of_sound;
    return travel_time / bin_count;
}

void Sonar::applyAdditionalGain(std::vector<float>& bins, float gain) {
    float gain_factor = gain / 0.5;
    std::transform(bins.begin(), bins.end(), bins.begin(), std::bind1st(std::multiplies<float>(), gain_factor));
    std::replace_if(bins.begin(), bins.end(), std::bind2nd(std::greater<float>(), 1.0), 1.0);
}
