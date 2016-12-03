#ifndef __IMAGING_SONAR_SIMULATION_HPP__
#define __IMAGING_SONAR_SIMULATION_HPP__

#include <Eigen/Core>

#include "Sonar.hpp"
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>
#include <frame_helper/FrameHelper.h>
#include <base/samples/Sonar.hpp>
#include <base/Angle.hpp>

namespace gpu_sonar_simulation
{

class SonarSimulation
{
public:

    SonarSimulation(float range,  float gain, uint32_t bin_count, uint32_t beam_count, base::Angle beam_width, base::Angle beam_height, uint value, bool isHeight, osg::ref_ptr<osg::Group> root);
    ~SonarSimulation();
    base::samples::Sonar simulateSonarData(const Eigen::Affine3d& sonar_pose);
    void updateSonarPose(const Eigen::Affine3d pose); 
    void processShader(osg::ref_ptr<osg::Image>& osg_image, std::vector<float>& bins);
    void setupShader(uint value, bool isHeight);
    
    base::samples::frame::Frame getLastFrame();

    void setSonarBinCount(uint32_t bin_count);
    uint32_t getSonarBinCount();
    
    void setSonarBeamCount(uint32_t beam_count);
    uint32_t getSonarBeamCount();
    
    void setSonarBeamWidth(base::Angle beam_width);
    base::Angle getSonarBeamWidth();

    void setRange(float range);
    void setGain(float gain_value);
    
    double gain;
    cv::Mat last_cv_image;

    gpu_sonar_simulation::Sonar sonar;
    vizkit3d_normal_depth_map::NormalDepthMap normal_depth_map;
    vizkit3d_normal_depth_map::ImageViewerCaptureTool capture_tool;
};

}

#endif
