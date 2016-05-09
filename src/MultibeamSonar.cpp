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
std::vector<float> MultibeamSonar::codeSonarData(const cv::Mat3f& cv_image) {

	std::vector<cv::Mat> shader;
	cv::split(cv_image, shader);

	// associates shader columns with their respective beam
	std::vector<float> sonar_data(_number_of_beams * _number_of_bins, 0.0);

	int middle_img = shader[0].cols * 0.5;

	for (int i = 0; i < shader[0].cols; i++) {

		// Checks normal values in each column
		if (cv::countNonZero(shader[0].col(i))) {
			int col_start = i;

			// gets the maximum angle in that column
			double max_angle;
			cv::minMaxIdx(shader[2].col(i), NULL, &max_angle);
			if (i < middle_img)
			    max_angle = (1 - max_angle) * 0.5;
			else
			    max_angle = (1 + max_angle) * 0.5;

			int id_beam = (_number_of_beams - 1) * max_angle;
			int id_curr = id_beam;

			// checks which columns belongs to same beam
			while ((id_curr == id_beam) && (i < (shader[2].cols - 1))) {
				cv::minMaxIdx(shader[2].col(++i), NULL, &max_angle);
				if (i < middle_img)
				    max_angle = (1 - max_angle) * 0.5;
				else
				    max_angle = (1 + max_angle) * 0.5;
				id_curr = (_number_of_beams - 1) * max_angle;
			}

			// gets the ROI (beam) of shader image
			cv::Mat cv_roi = cv_image.colRange(col_start, i);

			// processes shader informations
			std::vector<float> raw_intensity = decodeShaderImage(cv_roi);
			for (int i = 0; i < _number_of_bins; ++i)
				sonar_data[_number_of_bins * id_beam + i] = raw_intensity[i];
		}
	}

	return sonar_data;
}


}
