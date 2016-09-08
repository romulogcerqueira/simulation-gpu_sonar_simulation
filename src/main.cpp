#include <iostream>

// Rock includes
#include <gpu_sonar_simulation/Sonar.hpp>
#include <gpu_sonar_simulation/Utils.hpp>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace gpu_sonar_simulation;
using namespace cv;

#define myrand ((float)(random())/(float)(RAND_MAX) )

// create a depth and normal matrixes to test
cv::Mat createRandomImage(int rows, int cols) {
    cv::Mat raw_image = cv::Mat::zeros(rows, cols, CV_32FC3);

    for (int k = 0; k < raw_image.channels() - 1; k++)
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                raw_image.at<Vec3f>(i, j)[k] = myrand;

    return raw_image;
}

int main(int argc, char* argv[])
{
    /* generate a random shader image */
    uint width = 5144;
    base::Angle beam_width = base::Angle::fromDeg(120.0);
    base::Angle beam_height = base::Angle::fromDeg(20.0);
    uint height = width * tan(beam_height.rad * 0.5) / tan(beam_width.rad * 0.5);
    cv::Mat cv_shader = createRandomImage(height, width);

    base::Time ts0 = base::Time::now();

    /* initialize Sonar Simulation */
    uint32_t bin_count = 500;
    uint32_t beam_count = 256;
    Sonar sonar_sim(bin_count, beam_count, beam_width, beam_height);

    /* simulate sonar image */
    std::vector<float> bins;
    sonar_sim.decodeShader(cv_shader, bins);

    /* apply additional gain */
    float gain = 0.5;
    sonar_sim.applyAdditionalGain(bins, gain);

    /* encapsulate in rock's sonar structure */
    base::samples::Sonar sonar = sonar_sim.simulateSonar(bins, 50);

    std::cout << "Time: " << (base::Time::now() - ts0).toSeconds() << std::endl;

    return 0;
}
