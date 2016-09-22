#include <iostream>
#include <numeric>

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
    uint width = 3352;
    base::Angle beam_width = base::Angle::fromDeg(120.0);
    base::Angle beam_height = base::Angle::fromDeg(20.0);
    uint height = width * tan(beam_height.rad * 0.5) / tan(beam_width.rad * 0.5);
    cv::Mat cv_shader = createRandomImage(height, width);

    /* initialize Sonar Simulation */
    uint32_t bin_count = 1000;
    uint32_t beam_count = 256;
    Sonar sonar_sim(bin_count, beam_count, beam_width, beam_height);

    /* process 200 times */
    std::vector<double> timestamp;
    for (size_t i = 0; i < 200; i++) {
        base::Time ts0 = base::Time::now();

        /* simulate sonar image */
        std::vector<float> bins;
        sonar_sim.decodeShader(cv_shader, bins);

        /* apply additional gain */
        float gain = 0.5;
        sonar_sim.applyAdditionalGain(bins, gain);

        /* encapsulate in rock's sonar structure */
        float range = 50.0;
        base::samples::Sonar sonar = sonar_sim.simulateSonar(bins, range);

        /* accumulate timestamps */
        timestamp.push_back((base::Time::now() - ts0).toSeconds());
    }

    /* Performance results */
    double sum = std::accumulate(timestamp.begin(), timestamp.end(), 0.0);
    double mean = sum / timestamp.size();

    double accum = 0.0;
    for (size_t i = 0; i < timestamp.size(); i++)
        accum += (timestamp[i] - mean) * (timestamp[i] - mean);
    double stddev = sqrt(accum / (timestamp.size() - 1));

    std::cout << "=== MEAN  : " << mean << std::endl;
    std::cout << "=== STDDEV: " << stddev << std::endl;

    return 0;
}
