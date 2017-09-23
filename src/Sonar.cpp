#include "Sonar.hpp"

// Boost includes
#include <boost/random.hpp>

using namespace gpu_sonar_simulation;
using namespace cv;

void Sonar::decodeShader(const cv::Mat& cv_image, std::vector<float>& bins, bool enable_noise) {
    bins.resize(beam_count * bin_count);

    // check if beam_cols must be recalculated
    if (last_sonar.bin_count != bin_count || last_sonar.beam_count != beam_count) {
        beam_cols.clear();
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

    std::vector<float> raw_intensity;
    cv::Mat cv_roi;
    for (unsigned int beam_idx = 0; beam_idx < beam_count; ++beam_idx) {
        cv_image(cv::Rect(beam_cols[beam_idx * 2], 0, beam_cols[beam_idx * 2 + 1] - beam_cols[beam_idx * 2], cv_image.rows)).copyTo(cv_roi);
        convertShader(cv_roi, raw_intensity);
        if (enable_noise)
            applySpeckleNoise(raw_intensity, 0.4, 0.15);
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

    last_sonar = sonar;
    return sonar;
}

void Sonar::convertShader(cv::Mat& cv_image, std::vector<float>& bins) {
    // calculate depth histogram
    std::vector<int> bins_depth(bin_count, 0);
    float* ptr = reinterpret_cast<float*>(cv_image.data);
    for (int i = 0; i < cv_image.cols * cv_image.rows; i++) {
        int bin_idx = ptr[i * 3 + 1] * (bin_count - 1);
        bins_depth[bin_idx]++;
    }

    // calculate bins intesity using normal values, depth histogram and sigmoid function
    bins.assign(bin_count, 0.0);
    for (int i = 0; i < cv_image.cols * cv_image.rows; i++) {
        int bin_idx = ptr[i * 3 + 1] * (bin_count - 1);
        float intensity = (1.0 / bins_depth[bin_idx]) * sigmoid(ptr[i * 3]);
        bins[bin_idx] += intensity;
    }
}

void Sonar::applySpeckleNoise(std::vector<float>& bins, float mean, float stddev) {
    // produce speckle noise using a gaussian distribution
    unsigned long seed = base::Time::now().toMicroseconds();
    boost::random::mt19937 engine(seed);
    boost::random::normal_distribution<float> dist(mean, stddev);

    // apply noise to sonar data
    for (size_t i = 0; i < bins.size(); i++) {
        float noised = bins[i] * dist(engine);
        bins[i] = noised < 0 ? 0 : noised;
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
