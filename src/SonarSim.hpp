#ifndef _SONARSIM_HPP_
#define _SONARSIM_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>

using namespace cv;
using namespace std;

namespace gpu_sonar_simulation {
class SonarSim {
public:
	SonarSim();
	~SonarSim();
	std::vector<uint8_t> decodeRawImage(cv::Mat raw_image, int slices=40);


private:
	float sigmoid(float x);
	std::vector<uint8_t> getPingIntensity(cv::Mat hist);

	int _num_bins;
	float _range;
};

} // end namespace gpu_sonar_simulation

#endif
