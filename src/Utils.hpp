#ifndef _UTILS_HPP_
#define _UTILS_HPP_

// Opencv includes
#include <opencv2/core/core.hpp>

// Openscenegraph includes
#include <osg/Image>

namespace gpu_sonar_simulation{
    /**
    *  Convert the shader image from Openscenegraph to Opencv
    *  @param osg_image: the osg image (source)
    *  @param cv_image: the opencv image in cv::Mat (destination)
    */
    void convertOSG2CV(osg::ref_ptr<osg::Image>& osg_image, cv::Mat& cv_image);
}
#endif
