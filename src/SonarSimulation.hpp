#ifndef __IMAGING_SONAR_SIMULATION_HPP__
#define __IMAGING_SONAR_SIMULATION_HPP__

#include <Eigen/Core>

#include "Sonar.hpp"
#include <normal_depth_map/NormalDepthMap.hpp>
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <frame_helper/FrameHelper.h>
#include <base/samples/Sonar.hpp>
#include <base/Angle.hpp>

namespace gpu_sonar_simulation
{

class SonarSimulation
{
public:
    SonarSimulation(float range,  float gain, uint32_t bin_count,  
        base::Angle beam_width, base::Angle beam_height, unsigned int resolution, 
        bool isHeight, osg::ref_ptr<osg::Group> root, uint32_t beam_count = 0);
    ~SonarSimulation();

    /**
     * Generate a sonar sample response of a sonar at a given pose
     * @param pose: pose of the auv]
    */
    base::samples::Sonar simulateSonarData(const Eigen::Affine3d& sonar_pose);

    /**
    *  Process shader image in bins intensity.
    *  @param osg_image: the shader image (normal, depth and angle informations) in osg::Image format
    *  @param bins: the output simulated sonar data (all beams) in float
    */
    void processShader(osg::ref_ptr<osg::Image>& osg_image, std::vector<float>& bins);
    void setupShader(unsigned int resolution, bool isHeight);
    
    base::samples::frame::Frame getLastFrame();

    void setAttenuationCoefficient( const double frequency,
                                      const double temperature,
                                      const double depth,
                                      const double salinity,
                                      const double acidity);

    void setSonarBinCount(uint32_t bin_count);
    uint32_t getSonarBinCount();
    
    void setSonarBeamCount(uint32_t beam_count);
    uint32_t getSonarBeamCount();
    
    void setSonarBeamWidth(base::Angle beam_width);
    base::Angle getSonarBeamWidth();
    
    void setSonarBeamHeight(base::Angle beam_height);
    base::Angle getSonarBeamHeight();

    void enableSpeckleNoise(bool enable);

    void setRange(float range);
    float getRange();
    
    void setGain(float gain_value);
    float getGain();

private:    
    float gain;
    float range;
    cv::Mat last_cv_image;
    bool speckle_noise;
    unsigned int resolution;
    bool isHeight;

    gpu_sonar_simulation::Sonar sonar;
    normal_depth_map::NormalDepthMap normal_depth_map;
    normal_depth_map::ImageViewerCaptureTool capture_tool;
    
    /**
     *  Update sonar pose according to auv pose.
     *  @param pose: pose of the auv
    */
    void updateSonarPose(const Eigen::Affine3d pose); 
};

}

#endif
