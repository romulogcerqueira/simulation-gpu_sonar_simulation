/*
 * MicronSim_test.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: romulo
 */

#include <gpu_sonar_simulation/SonarSim.hpp>
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

//draw the scene with a small ball in the center with a big cube, cylinder and cone in back
void makeSimpleScene1(osg::ref_ptr<osg::Group> root) {

    osg::Geode *sphere = new osg::Geode();
    sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
    root->addChild(sphere);

    osg::Geode *cylinder = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(30, 0, 10), 10, 10)));
    root->addChild(cylinder);

    osg::Geode *cone = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 30, 0), 10, 10)));
    root->addChild(cone);

    osg::Geode *box = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -30, -10), 10)));
    root->addChild(box);
}

void viewPointsFromScene1(std::vector<osg::Vec3d>* eyes, std::vector<osg::Vec3d>* centers, std::vector<osg::Vec3d>* ups) {

    // view1 - near from the ball with the cylinder in back
    eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
    centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
    ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));

    // view2 - near from the ball with the cube in back
    eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
    centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
    ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));

    // view3 - near the cone in up side
    eyes->push_back(osg::Vec3d(-10.6743, 38.3461, 26.2601));
    centers->push_back(osg::Vec3d(-10.3734, 38.086, 25.3426));
    ups->push_back(osg::Vec3d(0.370619, -0.854575, 0.36379));

    // view4 - Faced the cube plane
    eyes->push_back(osg::Vec3d(0.0176255, -56.5841, -10.0666));
    centers->push_back(osg::Vec3d(0.0176255, -55.5841, -10.0666));
    ups->push_back(osg::Vec3d(0, 0, 1));
}


BOOST_AUTO_TEST_CASE(first_test) {

//	using namespace gpu_sonar_simulation_MicronSim;
//
//	int rows = 640, cols = 480;
//
//	SonarSim sonar;
//
//	// create a random image
//	cv::Mat raw_image = createRandomImage(rows, cols);
//
//	// print depth and normal matrixes
////	for (int i = 0; i < rows; i++)
////		for (int j = 0; j < cols; j++)
////			printf("D[%d][%d] = %.3f\n", i, j, raw_image.at<Vec3f>(i, j)[0]);
////	printf("\n");
////	for (int i = 0; i < rows; i++)
////		for (int j = 0; j < cols; j++)
////			printf("N[%d][%d] = %.3f\n", i, j, raw_image.at<Vec3f>(i, j)[1]);
//
//	cv::Mat raw_intensity = sonar.decodeRawImage(raw_image);
//	std::vector<uint8_t> beam = sonar.getPingIntensity(raw_intensity);

	// print beam data
//	printf("\nBeam Data:\n");
//	for (int i = 0; i < beam.size(); i++)
//		printf("beam[%d] = %d\n", i, beam[i]);

}


BOOST_AUTO_TEST_CASE(testIntegrationWithNormalDepthMap_testCase) {

	std::vector<osg::Vec3d> eyes, centers, ups;
	SonarSim sonar;

	float maxRange = 75;
	uint width = 640, height = 480;
	NormalDepthMap normalDepthMap(maxRange);
	ImageViewerCaptureTool capture(width, height);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	osg::ref_ptr<osg::Group> root = new osg::Group();
	viewPointsFromScene1(&eyes, &centers, &ups);
	makeSimpleScene1(root);
	root = normalDepthMap.applyShaderNormalDepthMap(root);

	for (uint i = 0; i < eyes.size(); ++i) {
		capture.setCameraPosition(eyes[i], centers[i], ups[i]);
		osg::ref_ptr<osg::Image> osgImage = capture.grabImage(root);
		cv::Mat3f cvImage(osgImage->t(), osgImage->s());
		cvImage.data = osgImage->data();
		cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
		cv::flip(cvImage, cvImage, 0);

		cv::Mat raw_intensity = sonar.decodeRawImage(cvImage);
		std::vector<uint8_t> beam = sonar.getPingIntensity(raw_intensity);

		for (uint j= 0; j < beam.size(); j++)
			printf("beam[%d] = %d\n", j, beam[j]);

		cv::imshow("Image",cvImage);
		cv::waitKey();
	}
}

BOOST_AUTO_TEST_SUITE_END();
