#define BOOST_TEST_MODULE "Sonar_test"
#include <boost/test/unit_test.hpp>

// Rock includes
#include <gpu_sonar_simulation/Sonar.hpp>
#include <gpu_sonar_simulation/Utils.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <base/Angle.hpp>

// Openscenegraph includes
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

// Opencv includes
#include <opencv2/highgui/highgui.hpp>

using namespace gpu_sonar_simulation;
using namespace normal_depth_map;
using namespace cv;

BOOST_AUTO_TEST_SUITE(gpu_sonar_simulation_MicronSim)

#define myrand ((float)(random())/(float)(RAND_MAX) )

// create a depth and normal matrixes to test
cv::Mat createRandomImage(int rows, int cols) {
    cv::Mat raw_image = cv::Mat::zeros(rows, cols, CV_32FC3);

    for (int k = 0; k < raw_image.channels(); k++)
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                raw_image.at<Vec3f>(i, j)[k] = myrand;

    return raw_image;
}

// add an oil rig manifold to the scene
void addOilRig(osg::ref_ptr<osg::Group> root){
    std::string current_path(__FILE__);
    current_path = current_path.substr(0, current_path.find_last_of("/"));
    osg::Node* oilring = osgDB::readNodeFile(current_path + "/oil_rig_manifold/visual.dae.osgb");

    osg::Matrix mtransf;
    mtransf.preMult(osg::Matrix::translate(0, 13, 0));
    mtransf.preMult(osg::Matrix::scale(0.1f,0.1f,0.1f));
    osg::MatrixTransform *ptransform = new osg::MatrixTransform();
    ptransform->setMatrix(mtransf);
    ptransform->addChild(oilring);

    root->addChild(ptransform);
}

// plot the accumulated bins intensity as an histogram
cv::Mat plotHistogram(const cv::Mat& raw_intensity, int bins) {
    double max;
    cv::minMaxIdx(raw_intensity, NULL, &max);

    cv::Mat1f new_bin = raw_intensity / max;
    cv::Mat cv_hist = cv::Mat3b::zeros(bins, bins);

    for (int i = 0; i < bins; i++) {
        cv::Scalar color;
        (i % 2) == 0 ? color = cv::Scalar(255, 255, 0) : color = cv::Scalar(0, 0, 255);
        cv::line(cv_hist, cv::Point(i, bins), cv::Point(i, bins * (1 - new_bin.at<float>(i))), color, 1);
    }

    return cv_hist;
}

BOOST_AUTO_TEST_CASE(histogram) {
    cv::Mat cv_image = createRandomImage(100, 100);
    cv_image = plotHistogram(cv_image, 256);
    cv::imshow("Normal histogram", cv_image);
    cv::waitKey(0);
}

BOOST_AUTO_TEST_CASE(complete_rotate_image) {
    osg::ref_ptr<osg::Image> osg_image;

    // init normal depth map
    double range = 60.0;
    NormalDepthMap normal_depth_map(range);

    // add oilrig
    osg::ref_ptr<osg::Group> root = new osg::Group();
    addOilRig(root);
    normal_depth_map.addNodeChild(root);

    // image viewer
    ImageViewerCaptureTool capture(640, 480);
    capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

    base::Angle rot = base::Angle::fromDeg(0.0);
    base::Angle transX = base::Angle::fromDeg(0.0);
    base::Angle transY = base::Angle::fromDeg(0.0);
    base::Angle transZ = base::Angle::fromDeg(0.0);

    while(true) {
        // transformation matrixes
        osg::Matrixd matrix;
        matrix.setTrans(osg::Vec3(transX.getDeg(), transY.getDeg(), transZ.getDeg()));
        matrix.setRotate(osg::Quat(rot.getRad(), osg::Vec3(0, 0, 1)));
        matrix.invert(matrix);

        osg::Matrixd rock_coordinate_system =
            osg::Matrixd::rotate(-M_PI_2, osg::Vec3(0, 0, -1)) *
            osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

        osg::Matrixd m = matrix * rock_coordinate_system;
        osg::Vec3 eye, center, up;
        m.getLookAt(eye, center, up);
        capture.setCameraPosition(eye, center, up);

        // grab capture
        osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normal_depth_map.getNormalDepthMapNode());
        cv::Mat cvImage;
        convertOSG2CV(osgImage, cvImage);

        cv::imshow("Normal Depth Map", cvImage);
        char k = cv::waitKey(0);

        switch(k) {
            case 27:		//	ESC
                exit(0);
                break;
            case 85:		// UP
                transZ += base::Angle::fromDeg(2.0);
                break;
            case 86:		// DOWN
                transZ -= base::Angle::fromDeg(2.0);
                break;
            case 81:		// LEFT
                transY += base::Angle::fromDeg(2.0);
                break;
            case 83:		// RIGHT
                transY -= base::Angle::fromDeg(2.0);
                break;
            case 82:		// PAGE UP
                transX += base::Angle::fromDeg(2.0);
                break;
            case 84:		// PAGE DOWN
                transX -= base::Angle::fromDeg(2.0);
                break;
            case 122:		// Z
                rot += base::Angle::fromDeg(5.0);
                break;
            case 120:		// X
                rot -= rot += base::Angle::fromDeg(5.0);
                break;
        }

        std::cout << "--- camera params ---" << std::endl;
        std::cout << "trans  : " << transX.getDeg() << "," << transY.getDeg() << "," << transZ.getDeg() << std::endl;
        std::cout << "eye    : " << eye.x() << "," << eye.y() << "," << eye.z() << std::endl;
        std::cout << "center : " << center.x() << "," << center.y() << "," << center.z() << std::endl;
        std::cout << "up     : " << up.x() << "," << up.y() << "," << up.z() << std::endl;
    }

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(0,0,600,600);
    viewer.setSceneData(normal_depth_map.getNormalDepthMapNode());
    viewer.run();
}
BOOST_AUTO_TEST_SUITE_END();
