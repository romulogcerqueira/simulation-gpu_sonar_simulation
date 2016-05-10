#include "MultibeamSonar.hpp"
#include <iostream>

using namespace gpu_sonar_simulation;

namespace gpu_sonar_simulation {

// Simulate a base::samples::SonarBeam data and update the sonar head position
base::samples::Sonar MultibeamSonar::simulateMultiBeam(const std::vector<float>& data) {

    base::samples::Sonar sonar;

    sonar.time = base::Time::now();
    sonar.speed_of_sound = _speed_of_sound;
    sonar.bin_count = _number_of_bins;
    sonar.beam_count = _number_of_beams;
    sonar.beam_width = _beam_width;
    sonar.beam_height = _beam_height;
    sonar.bins = data;

    base::Angle angular_resolution = base::Angle::fromRad(sonar.beam_width.rad / sonar.beam_count);
    sonar.setRegularBeamBearings(base::Angle::fromRad(-sonar.beam_width.rad / 2), angular_resolution);

    sonar.validate();
    return sonar;
}

// Split image in beam parts. The shader is not radially spaced equally
// over the FOV-X degree sector, so it is necessary to identify which column
// is contained on each beam.
std::vector<float> MultibeamSonar::codeSonarData(const cv::Mat3f& cv_image)
{
    std::vector<cv::Mat> shader;
    cv::split(cv_image, shader);

    // associates shader columns with their respective beam
    std::vector<float> sonar_data(_number_of_beams * _number_of_bins, 0.0);

    double const half_fovx = _number_of_beams * _beam_width.getRad() / 2;
    double const half_width = static_cast<double>(shader[0].cols) / 2;
    double const angle2x = half_width * tan(half_fovx);
    for (unsigned int beam_idx = 0; beam_idx < _number_of_beams; ++beam_idx)
    {
        int min_col = round(half_width + tan(- half_fovx + beam_idx * _beam_width.getRad()) * angle2x);
        int max_col = round(half_width + tan(- half_fovx + (beam_idx + 1) * _beam_width.getRad()) * angle2x);
        cv::Mat cv_roi = cv_image.colRange(min_col, max_col);

        std::vector<float> raw_intensity = decodeShaderImage(cv_roi);
        for (int j = 0; j < _number_of_bins; j++)
            sonar_data[_number_of_bins * beam_idx + j] = raw_intensity[j];
    }
    return sonar_data;
}


}
