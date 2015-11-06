/*
 * SonarUtils.hpp
 *
 *  Created on: Apr 29, 2015
 *      Author: romulogcerqueira
 */

#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <osg/Image>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/Angle.hpp>
#include <iostream>
#include <sstream>

namespace gpu_sonar_simulation{
	cv::Mat convertShaderOSG2CV(osg::ref_ptr<osg::Image> osg_image);
	void applyDynamicRangeControl(std::vector<uint8_t>& data, uint8_t ad_low, const uint8_t ad_span);
	cv::Mat plotNormalHistogram(const cv::Mat& raw_image, int bins);
	cv::Mat plotDepthHistogram(const cv::Mat& raw_image, int bins);
}
#endif
