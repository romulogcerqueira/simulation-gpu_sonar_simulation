/*
 * MicronSim_test.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: romulo
 */

#include <gpu_sonar_simulation/ScanSonar.hpp>
#include <gpu_sonar_simulation/SonarUtils.hpp>
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>

#include <iostream>
#include <stdio.h>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define BOOST_TEST_MODULE "SonarSim_test"
#include <boost/test/unit_test.hpp>

using namespace gpu_sonar_simulation;
using namespace vizkit3d_normal_depth_map;

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

//draw the scene with a small ball in the center with a big cube, Box and cone in back
void makeSampleScene(osg::ref_ptr<osg::Group> root) {

    osg::Geode *object = new osg::Geode();

	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-10,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3( 20,0,0), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,30,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(-40,0,0), 1, 1)));

	root->addChild(object);
}


//BOOST_AUTO_TEST_CASE(first_test_case) {
//
//	using namespace gpu_sonar_simulation_MicronSim;
//
//	SonarConfig config;
//	ScanSonar sonar;
//
//	sonar.setup(config);
//
//	sonar.setNumberOfBins(5);
//
//
//	cv::Mat raw_image = createRandomImage(4, 4);
//
//	vector<cv::Mat> channels(3);
//	split(raw_image, channels);
//
//	std::cout << "Depth: " << channels[0] << std::endl;
//	std::cout << "Normal: " << channels[1] << std::endl;
//
//
//	cv::Mat raw_intensity = sonar.decodeRawImage2(raw_image);
//
//	std::cout << "Raw intensity: " << raw_intensity << std::endl;
//
//	std::vector<uint8_t> data = sonar.getPingIntensity(raw_intensity);
//
//	base::samples::SonarBeam sonar_beam = sonar.simulateSonarBeam(data);
//}

//BOOST_AUTO_TEST_CASE(drivers_integration) {
//
//	using namespace gpu_sonar_simulation_MicronSim;
//
//	SonarConfig config;
//	ScanSonar sonar;
//
//	sonar.setup(config);
//
//	cv::Mat raw_image = createRandomImage(640, 480);
//
//	cv::Mat raw_intensity = sonar.decodeRawImage(raw_image);
//
//	std::vector<uint8_t> data = sonar.getPingIntensity(raw_intensity);
//
//	base::samples::SonarBeam sonar_beam = sonar.simulateSonarBeam(data);
//}

BOOST_AUTO_TEST_CASE(complete_rotate_image) {

	osg::ref_ptr<osg::Image> osg_image;




	// ======================== INIT SCENE ========================

	uint width = 640, height = 480;
	double range = 50.0, degree = 1.0;

	ScanSonar sonar;
	sonar.setNumberOfBins(50);
	NormalDepthMap normal_depth_map(range);
	ImageViewerCaptureTool capture(width, height);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	osgViewer::Viewer viewer;

	osg::ref_ptr<osg::Group> root = new osg::Group();
	makeSampleScene(root);
	normal_depth_map.addNodeChild(root);


	// ======================= CORRECT SCENE =======================

	osg::Matrix m = capture.getViewMatrix();
	m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(90.0), osg::X_AXIS)));
	capture.setViewMatrix(m);

	// ======================= GRAB CAPTURE =======================

	bool loop = true;
	while(loop)
	{
		osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normal_depth_map.getNormalDepthMapNode());

		// ======================= SONAR SIMULATION =======================

		cv::Mat3f cvImage = convertShaderOSG2CV(osgImage);
		cv::Mat raw_intensity = sonar.decodeShaderImage(cvImage);
		std::cout << "Intensity: " << raw_intensity << std::endl;


		// ======================= ROTATE THE CAMERA =======================

		cv::imshow("teste", cvImage);

		char k = cv::waitKey(0);
		if (k == 27)
			loop = false;


		m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-degree), osg::Z_AXIS)));
		capture.setViewMatrix(m);

	}


}

BOOST_AUTO_TEST_SUITE_END();
