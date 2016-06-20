#include "Utils.hpp"

// Opencv includes
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace osg;

namespace gpu_sonar_simulation {

void convertOSG2CV(osg::ref_ptr<osg::Image>& osg_image, cv::Mat& cv_image) {

	cv_image.create(osg_image->t(), osg_image->s(), CV_32FC3);
	cv_image.data = osg_image->data();
	cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
	cv::flip(cv_image, cv_image, 0);
}
}
