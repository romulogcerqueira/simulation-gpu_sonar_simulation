/*
 * SonarUtils.hpp
 *
 *  Created on: Apr 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _SONARUTILS_HPP_
#define _SONARUTILS_HPP_

#include <osg/Image>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace gpu_sonar_simulation{
	cv::Mat 	convertShaderOSG2CV(osg::ref_ptr<osg::Image> osg_image);
	std::vector<uint8_t> applyDynamicRangeControl(std::vector<uint8_t> data, uint8_t ad_low, uint8_t ad_span);

}
#endif
