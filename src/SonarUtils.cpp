#include "SonarUtils.hpp"
#include <iostream>

using namespace cv;
using namespace osg;

namespace gpu_sonar_simulation {

// Convert the shader image from OSG to OpenCV
cv::Mat convertShaderOSG2CV(osg::ref_ptr<osg::Image> osg_image) {

	cv::Mat3f cvImage(osg_image->t(), osg_image->s());
	cvImage.data = osg_image->data();
	cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
	cv::flip(cvImage, cvImage, 0);

	return cvImage;
}

// Create a transformation matrix to simulate the 360º scanning sonar reader
osg::Matrix getScanMatrix(const osg::Matrix src, double deg) {
	osg::Matrix dst = src;

	dst.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-deg), osg::Z_AXIS)));

	return dst;
}

}
