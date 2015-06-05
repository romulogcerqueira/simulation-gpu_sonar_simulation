/*
 * SonarUtils.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: romulogcerqueira
 */

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

// The sonar receiver has an dynamic range (0..80dB = 0..255). Two parameters are used by
// Sonar in 'mtHeadCommand': ad_low, which sets the Lower boundary of the sampling window,
// it can be increased to make the Sonar display less sensitive and filter out background
// and receiver self noise; and ad_span sets the width of the sampling window and therefore
// acts as a Contrast control.
std::vector<uint8_t> applyDynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span){

	for(uint i=0; i<data.size(); i++)
	{
		if (data[i] <= ad_low)
			data[i] = 0;
		else if (data[i] >= ad_low + ad_span)
			data[i] = 255;
		else
			data[i] = (uint8_t) ((double)(data[i]-ad_low)*255.0/(double)ad_span);
	}
	return data;
}

}
