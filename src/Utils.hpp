#ifndef _UTILS_HPP_
#define _UTILS_HPP_

// Opencv includes
#include <opencv2/core/core.hpp>

// Openscenegraph includes
#include <osg/Image>
#include <cmath>

namespace gpu_sonar_simulation{
    /**
    *  Convert the shader image from Openscenegraph to Opencv
    *  @param osg_image: the osg image (source)
    *  @param cv_image: the opencv image in cv::Mat (destination)
    */
    void convertOSG2CV(osg::ref_ptr<osg::Image>& osg_image, cv::Mat& cv_image);
   
    /**
     * @brief compute Underwater Signal Attenuation coefficient
     *
     *  This method is based on paper "A simplified formula for viscous and
     *  chemical absorption in sea water". The method computes the attenuation
     *  coefficient that will be used on shader normal intensite return.
     *
     *  @param double frequency: sound frequency in kHz.
     *  @param double temperature: water temperature in Celsius degrees.
     *  @param double depth: distance from water surface in meters.
     *  @param double salinity: amount of salt dissolved in a body of water in ppt.
     *  @param double acidity: pH water value.
     *
     *  @return double coefficient attenuation value
     */

    double underwaterSignalAttenuation( const double frequency,
                                        const double temperature,
                                        const double depth,
                                        const double salinity,
                                        const double acidity);
}
#endif
