// C++ includes
#include <iostream>
#include <numeric>

// Rock includes
#include <gpu_sonar_simulation/Sonar.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <gpu_sonar_simulation/Utils.hpp>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// OpenSceneGraph includes
#include <osg/Group>
#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osg/ShapeDrawable>
#include <osg/Geode>

using namespace gpu_sonar_simulation;
using namespace cv;

// generate arbitrary value between min and max
float randomValue(float min, float max)
{
    return (min + (float)rand() / (RAND_MAX + 1.0f) * (max - min));
}

// generate arbitrary vector between min and max
osg::Vec3 randomVector(float min_x, float max_x,
                       float min_y, float max_y,
                       float min_z, float max_z)
{
    return osg::Vec3(randomValue(min_x, max_x),
                     randomValue(min_y, max_y),
                     randomValue(min_z, max_z));
}

// draw the scene with box, sphere, cylinder and cone
osg::ref_ptr<osg::Group> demoScene(
    float min_x, float max_x,
    float min_y, float max_y,
    float min_z, float max_z)
{
    const float radius = 0.8f;
    const float height = 1.0f;
    osg::TessellationHints *hints = new osg::TessellationHints();
    hints->setDetailRatio(2.0f);

    osg::Geode *geode = new osg::Geode();
    osg::ShapeDrawable *shape = new osg::ShapeDrawable();
    osg::Vec3 pos;

    pos = randomVector(min_x, max_x, min_y, max_y, min_z, max_z);
    shape = new osg::ShapeDrawable(new osg::Sphere(pos, radius), hints);
    geode->addDrawable(shape);

    pos = randomVector(min_x, max_x, min_y, max_y, min_z, max_z);
    shape = new osg::ShapeDrawable(new osg::Box(pos, 2 * radius), hints);
    geode->addDrawable(shape);

    pos = randomVector(min_x, max_x, min_y, max_y, min_z, max_z);
    shape = new osg::ShapeDrawable(new osg::Cone(pos, radius, height), hints);
    geode->addDrawable(shape);

    pos = randomVector(min_x, max_x, min_y, max_y, min_z, max_z);
    shape = new osg::ShapeDrawable(new osg::Cylinder(pos, radius, height), hints);
    geode->addDrawable(shape);

    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(geode);
    return root;
}

void updateSonarPose(normal_depth_map::ImageViewerCaptureTool &capture, const Eigen::Affine3d pose)
{
    // convert OSG (Z-forward) to RoCK coordinate system (X-forward)
    static const osg::Matrixd rock_coordinate_matrix = osg::Matrixd::rotate(M_PI_2, osg::Vec3(0, 0, 1)) * osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

    // transformation matrixes multiplication
    osg::Matrixd matrix(pose.data());
    matrix.invert(matrix);

    // correct coordinate system and apply geometric transformations
    osg::Matrixd m = matrix * rock_coordinate_matrix;

    osg::Vec3 eye, center, up;
    m.getLookAt(eye, center, up);
    capture.setCameraPosition(eye, center, up);
}

void processShader(gpu_sonar_simulation::Sonar &sonar_sim, normal_depth_map::ImageViewerCaptureTool &capture, osg::ref_ptr<osg::Image> &osg_image, std::vector<float> &bins, float gain)
{
    // receives shader image in opencv format
    cv::Mat cv_image, cv_depth;
    gpu_sonar_simulation::convertOSG2CV(osg_image, cv_image);
    osg::ref_ptr<osg::Image> osg_depth = capture.getDepthBuffer();
    gpu_sonar_simulation::convertOSG2CV(osg_depth, cv_depth);

    // replace depth matrix
    std::vector<cv::Mat> channels;
    cv::split(cv_image, channels);
    channels[1] = cv_depth;
    cv::merge(channels, cv_image);

    // decode shader informations to sonar data
    sonar_sim.decodeShader(cv_image, bins);

    // apply the additional gain
    sonar_sim.applyAdditionalGain(bins, gain);

    // cv::imshow("cv_image", cv_image);
    // cv::waitKey();
}

void computePerformance(const std::vector<uint> &beam_counts,
                        const std::vector<uint> &bin_counts,
                        const std::vector<base::Angle> &beam_widths,
                        const std::vector<base::Angle> &beam_heights,
                        uint N,
                        float resolution_constant,
                        bool isScanning)
{
    float range = 20.0;
    float gain = 0.5;

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();

    for (size_t k = 0; k < beam_widths.size(); k++)
    {
        for (size_t i = 0; i < beam_counts.size(); i++)
        {
            for (size_t j = 0; j < bin_counts.size(); j++)
            {
                /* set parameters */
                uint32_t beam_count = beam_counts[i];
                uint32_t bin_count = bin_counts[j];
                uint width = bin_count * resolution_constant;
                base::Angle beam_width = beam_widths[k];
                base::Angle beam_height = beam_heights[k];

                /** Normal and depth map from an osg scene. */
                normal_depth_map::NormalDepthMap normal_depth_map;

                /** Capture the osg::Image from a node scene */
                normal_depth_map::ImageViewerCaptureTool capture;

                /* process N times */
                std::vector<double> timestamp;
                for (size_t m = 0; m < N; m++)
                {
                    base::Time ts0 = base::Time::now();
                    float value = bin_count * resolution_constant;

                    /* initialize shader (NormalDepthMap and ImageViewerCaptureTool) */
                    normal_depth_map = normal_depth_map::NormalDepthMap(range, beam_width.getRad() * 0.5, beam_height.getRad() * 0.5);
                    normal_depth_map.setMaxRange(range);
                    capture = normal_depth_map::ImageViewerCaptureTool(beam_height.getRad(), beam_width.getRad(), value, isScanning);
                    capture.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
                    updateSonarPose(capture, pose);
                    osg::ref_ptr<osg::Group> root = demoScene(0, range, -range * 0.5, range * 0.5, -range * 0.5, range * 0.5);
                    normal_depth_map.addNodeChild(root);

                    /* receives the shader image */
                    osg::ref_ptr<osg::Image> osg_image = capture.grabImage(normal_depth_map.getNormalDepthMapNode());

                    /*  process the shader image */
                    gpu_sonar_simulation::Sonar sonar_sim(bin_count, beam_count, beam_width, beam_height);
                    std::vector<float> bins;
                    processShader(sonar_sim, capture, osg_image, bins, gain);

                    /* simulate sonar reading */
                    base::samples::Sonar sonar = sonar_sim.simulateSonar(bins, range);

                    // set the sonar bearings
                    if (isScanning)
                    {
                        sonar.bearings.push_back(base::Angle::fromDeg(0.0));
                    }
                    else
                    {
                        base::Angle interval = base::Angle::fromRad(sonar_sim.beam_width.getRad() / sonar_sim.beam_count);
                        base::Angle start = base::Angle::fromRad(-sonar_sim.beam_width.getRad() / 2);
                        sonar.setRegularBeamBearings(start, interval);
                    }
                    sonar.validate();

                    /* accumulate timestamps */
                    timestamp.push_back((base::Time::now() - ts0).toSeconds());
                }

                /* Performance results */
                double sum = std::accumulate(timestamp.begin(), timestamp.end(), 0.0);
                double mean = sum / timestamp.size();

                double accum = 0.0;
                for (size_t m = 0; m < timestamp.size(); m++)
                    accum += (timestamp[m] - mean) * (timestamp[m] - mean);
                double stddev = sqrt(accum / (timestamp.size() - 1));
                double fps = 1 / mean;

                std::cout << "=== PERFORMANCE RESULTS ===" << std::endl;
                if (isScanning)
                    std::cout << "Mechanical Scanning Imaging Sonar" << std::endl;
                else
                    std::cout << "Forward Looking Sonar" << std::endl;
                std::cout << "FOV       = " << beam_width.getDeg() << " x " << beam_height.getDeg() << std::endl;
                std::cout << "Beams     = " << beam_count << std::endl;
                std::cout << "Bins      = " << bin_count << std::endl;
                std::cout << "Width     = " << width << std::endl;
                std::cout << "Mean      = " << mean << std::endl;
                std::cout << "Stddev    = " << stddev << std::endl;
                std::cout << "FPS       = " << fps << std::endl;
                std::cout << std::endl;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    srand(time(NULL));

    uint N = 500;

    /* FLS parameters */
    float resolution_constant = 2.56;
    std::vector<uint> beam_counts = {128, 256};
    std::vector<uint> bin_counts = {500, 1000};
    std::vector<base::Angle> beam_widths = {base::Angle::fromDeg(120.0), base::Angle::fromDeg(90.0)};
    std::vector<base::Angle> beam_heights = {base::Angle::fromDeg(20.0), base::Angle::fromDeg(15.0)};
    bool isScanning = false;
    computePerformance(beam_counts, bin_counts, beam_widths, beam_heights, N, resolution_constant, isScanning);

    /* MSIS parameters */
    // float resolution_constant = 1;
    // std::vector<uint> beam_counts = {1};
    // std::vector<uint> bin_counts = {500, 1000};
    // std::vector<base::Angle> beam_widths = {base::Angle::fromDeg(3.0), base::Angle::fromDeg(2.0)};
    // std::vector<base::Angle> beam_heights = {base::Angle::fromDeg(35.0), base::Angle::fromDeg(20.0)};
    // bool isScanning = true;
    // computePerformance(beam_counts, bin_counts, beam_widths, beam_heights, N, resolution_constant, isScanning);

    return 0;
}
