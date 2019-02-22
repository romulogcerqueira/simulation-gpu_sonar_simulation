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

float randomValue(float min, float max)
{
    return (min + (float)rand() / (RAND_MAX + 1.0f) * (max - min));
}

// create a depth and normal matrixes to test
cv::Mat createRandomImage(int rows, int cols)
{
    cv::Mat raw_image = cv::Mat::zeros(rows, cols, CV_32FC3);

    for (size_t k = 0; k < raw_image.channels() - 1; k++)
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                raw_image.at<Vec3f>(i, j)[k] = randomValue(0, 1);

    return raw_image;
}

int main(int argc, char *argv[])
{
    /* parameters */
    std::vector<uint> beam_counts = {128, 256};
    std::vector<uint> bin_counts = {500, 1000};
    std::vector<base::Angle> beam_widths = {base::Angle::fromDeg(120.0), base::Angle::fromDeg(90.0)};
    std::vector<base::Angle> beam_heights = {base::Angle::fromDeg(20.0), base::Angle::fromDeg(15.0)};

    uint N = 500;
    float resolution_constant = 2.56;

    // TODO: missing attenuation, reverb (both done on shader)

    for (size_t k = 0; k < beam_widths.size(); k++)
    {
        for (size_t i = 0; i < beam_counts.size(); i++)
        {
            for (size_t j = 0; j < bin_counts.size(); j++)
            {
                /* generate a random shader image */
                uint width = bin_counts[j] * resolution_constant;
                base::Angle beam_width = beam_widths[k];
                base::Angle beam_height = beam_heights[k];
                uint height = width * tan(beam_height.rad * 0.5) / tan(beam_width.rad * 0.5);
                cv::Mat cv_shader = createRandomImage(height, width);

                /* initialize Sonar Simulation */
                uint32_t bin_count = bin_counts[j];
                uint32_t beam_count = beam_counts[i];
                Sonar sonar_sim(bin_count, beam_count, beam_width, beam_height);

                /* process N times */
                std::vector<double> timestamp;
                for (size_t m = 0; m < N; m++)
                {
                    base::Time ts0 = base::Time::now();

                    /* simulate sonar image */
                    std::vector<float> bins;
                    bool enable_speckle_noise = true;
                    sonar_sim.decodeShader(cv_shader, bins, enable_speckle_noise);

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
                for (size_t m = 0; m < timestamp.size(); m++)
                    accum += (timestamp[m] - mean) * (timestamp[m] - mean);
                double stddev = sqrt(accum / (timestamp.size() - 1));

                std::cout << "=== PARAMETERS ===" << std::endl;
                std::cout << "FOV       = " << sonar_sim.beam_width.getDeg() << " x " << sonar_sim.beam_height.getDeg() << std::endl;
                std::cout << "Beams     = " << beam_count << std::endl;
                std::cout << "Bins      = " << bin_count << std::endl;
                std::cout << "Width     = " << width << std::endl;
                std::cout << "Mean      = " << mean << std::endl;
                std::cout << "Stddev    = " << stddev << std::endl;
                std::cout << std::endl;
            }
        }
    }

    return 0;
}
