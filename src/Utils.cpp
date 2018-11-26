#include "Utils.hpp"

// Opencv includes
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace osg;

namespace gpu_sonar_simulation {

void convertOSG2CV(osg::ref_ptr<osg::Image>& osg_image, cv::Mat& cv_image) {
    cv_image = cv::Mat(osg_image->t(), osg_image->s(), CV_32FC3, osg_image->data());
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
}
    double underwaterSignalAttenuation( const double frequency,
                                        const double temperature,
                                        const double depth,
                                        const double salinity,
                                        const double acidity ) 
    {
    
        double frequency2 = frequency * frequency;
    
        // borid acid and magnesium sulphate relaxation frequencies (in kHz)
        double f1 = 0.78 * pow(salinity / 35, 0.5) * exp(temperature / 26);
        double f2 = 42 * exp(temperature / 17);
    
        // borid acid contribution
        double borid = 0.106 * ((f1 * frequency2) / (frequency2 + f1 * f1)) * exp((acidity - 8) / 0.56);
    
        // magnesium sulphate contribuion
        double magnesium = 0.52 * (1 + temperature / 43) * (salinity / 35)
                            * ((f2 * frequency2) / (frequency2 + f2 * f2)) * exp(-depth / 6000);
    
        // freshwater contribution
        double freshwater = 0.00049 * frequency2 * exp(-(temperature / 27 + depth / 17000));
    
        // absorptium attenuation coefficient in dB/km
        double attenuation = borid + magnesium + freshwater;
    
        // convert dB/km to dB/m
        attenuation = attenuation / 1000.0;
    
        // convert dB/m to Pa/m
        attenuation = pow(10, -attenuation / 20);
        attenuation = -log(attenuation);
    
        return attenuation;
    }

}
