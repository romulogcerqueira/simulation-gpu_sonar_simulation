#include "SonarSimulation.hpp"
#include <gpu_sonar_simulation/Utils.hpp>
#include <sstream>

using namespace gpu_sonar_simulation;

SonarSimulation::SonarSimulation(float range, float gain, uint32_t bin_count,
        base::Angle beam_width, base::Angle beam_height,
        unsigned int resolution, bool isHeight, osg::ref_ptr<osg::Group> root, uint32_t beam_count):
        sonar(bin_count, beam_count, beam_width, beam_height),
        gain(gain),
        range(range),
        speckle_noise(false),
        resolution(resolution),
        isHeight(isHeight)
{
    // initialize shader (NormalDepthMap and ImageViewerCaptureTool)
    normal_depth_map = normal_depth_map::NormalDepthMap(range);
    normal_depth_map.addNodeChild(root);

    capture_tool = normal_depth_map::ImageViewerCaptureTool(sonar.beam_height.getRad(),
                                                            sonar.beam_width.getRad(), resolution, isHeight);
    capture_tool.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
}

SonarSimulation::~SonarSimulation()
{}

void SonarSimulation::processShader(osg::ref_ptr<osg::Image>& osg_image,
    std::vector<float>& bins) {
    // receives shader image in opencv format
    cv::Mat cv_image;
    gpu_sonar_simulation::convertOSG2CV(osg_image, cv_image);

    // decode shader informations to sonar data
    sonar.decodeShader(cv_image, bins, speckle_noise);
    last_cv_image = cv_image;

    // apply the additional gain
    sonar.applyAdditionalGain(bins, gain);
}

base::samples::frame::Frame SonarSimulation::getLastFrame()
{
    last_cv_image.convertTo(last_cv_image, CV_8UC3, 255);
    cv::flip(last_cv_image, last_cv_image, 0);

    base::samples::frame::Frame frame;
    frame_helper::FrameHelper::copyMatToFrame(last_cv_image, frame);
    return frame;
}


base::samples::Sonar SonarSimulation::simulateSonarData(const Eigen::Affine3d& sonar_pose)
{
    // update the sonar pose in the depth map
    updateSonarPose(sonar_pose);

    // receive shader image
    osg::ref_ptr<osg::Image> osg_image =
        capture_tool.grabImage(normal_depth_map.getNormalDepthMapNode());
    // process the shader image
    std::vector<float> bins;
    processShader(osg_image, bins);

    // simulate sonar data
    base::samples::Sonar sim_sonar_data = sonar.simulateSonar(bins, range);

    return sim_sonar_data;
}

void SonarSimulation::updateSonarPose(const Eigen::Affine3d pose)
{
    // convert OSG (Z-forward) to RoCK coordinate system (X-forward)
    static const osg::Matrixd rock_coordinate_matrix = osg::Matrixd::rotate( M_PI_2, osg::Vec3(0, 0, 1))
        * osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

    // transformation matrixes multiplication
    osg::Matrixd matrix(pose.data());
    matrix.invert(matrix);

    // correct coordinate system and apply geometric transformations
    osg::Matrixd m = matrix * rock_coordinate_matrix;

    osg::Vec3 eye, center, up;
    m.getLookAt(eye, center, up);
    capture_tool.setCameraPosition(eye, center, up);
}

void SonarSimulation::setupShader(unsigned int resolution, bool isHeight)
{
    capture_tool = normal_depth_map::ImageViewerCaptureTool(sonar.beam_height.getRad(),
        sonar.beam_width.getRad(), resolution, isHeight);
    capture_tool.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
}

void SonarSimulation::setAttenuationCoefficient( const double frequency,
    const double temperature, const double depth,
    const double salinity, const double acidity,
    bool enable)
{
    double attenuation_coeff = 0;
    if (enable)
        attenuation_coeff = underwaterSignalAttenuation(frequency, temperature, depth, salinity, acidity);
    normal_depth_map.setAttenuationCoefficient(attenuation_coeff);
}


void SonarSimulation::setSonarBinCount(uint32_t bin_count)
{
    sonar.bin_count = bin_count;
}

uint32_t SonarSimulation::getSonarBinCount()
{
    return sonar.bin_count;
}

void SonarSimulation::setSonarBeamCount(uint32_t beam_count)
{
    sonar.beam_count = beam_count;
}
uint32_t SonarSimulation::getSonarBeamCount()
{
    return sonar.beam_count;
}

void SonarSimulation::setSonarBeamWidth(base::Angle beam_width)
{
    sonar.beam_width = beam_width;
    setupShader(resolution, isHeight);
}
base::Angle SonarSimulation::getSonarBeamWidth()
{
    return sonar.beam_width;
}

void SonarSimulation::setSonarBeamHeight(base::Angle beam_height)
{
    sonar.beam_height = beam_height;
    setupShader(resolution, isHeight);
}
base::Angle SonarSimulation::getSonarBeamHeight()
{
    return sonar.beam_height;
}
void SonarSimulation::setGain(float gain_value)
{
    gain = gain_value;
}
float SonarSimulation::getGain()
{
    return gain;
}

void SonarSimulation::setRange(float range_value)
{
    normal_depth_map.setMaxRange(range_value);
    range = range_value;
}

float SonarSimulation::getRange()
{
    return range;
}

void SonarSimulation::enableSpeckleNoise(bool enable)
{
    speckle_noise = enable;
}

void SonarSimulation::enableReverb(bool enable)
{
    normal_depth_map.setDrawReverb(enable);
}