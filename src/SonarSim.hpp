#ifndef _MICRONSIM_HPP_
#define _MICRONSIM_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

namespace gpu_sonar_simulation {
class SonarSim {
public:
	cv::Mat decodeRawImage(cv::Mat raw_image, int num_bins, int slices);

private:
	cv::Mat getPingIntensity(cv::Mat hist);
};

} // end namespace gpu_sonar_simulation

#endif
