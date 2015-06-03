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

#include <osgDB/ReadFile>
#include <osg/Node>
#include <osg/Transform>

#include <osgOcean/Version>
#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/SiltEffect>
#include <osgOcean/ShaderManager>


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

	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-20,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3( 40,0,0), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,60,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(-80,0,0), 1, 1)));


	root->addChild(object);
}

//draw the scene with a small ball in the center with a big cube, Box and cone in back
void makeManifold(osg::ref_ptr<osg::Group> root) {

	osg::Node* manifold = osgDB::readNodeFile("/home/romulogc/flatfish_meshs/simple_manifold/model.dae");

	osg::Matrix mtransf;
	mtransf.preMult(osg::Matrix::scale(0.01f,0.01f,0.01f));
	osg::MatrixTransform *ptransform = new osg::MatrixTransform();
	ptransform->setMatrix(mtransf);
	ptransform->addChild(manifold);

	root->addChild(ptransform);
}


BOOST_AUTO_TEST_CASE(first_test_case) {

	using namespace gpu_sonar_simulation_MicronSim;

	ScanSonar sonar;

	sonar.setNumberOfBins(5);

	cv::Mat raw_image = createRandomImage(4, 4);

	vector<cv::Mat> channels(3);
	split(raw_image, channels);

	cv::Mat raw_intensity = sonar.decodeShaderImage(raw_image);

	std::vector<uint8_t> data = sonar.getPingData(raw_intensity);

	base::samples::SonarBeam sonar_beam = sonar.simulateSonarBeam(data, 1.8);
}


BOOST_AUTO_TEST_CASE(complete_rotate_image) {

	osg::ref_ptr<osg::Image> osg_image;

	// ======================== INIT SCENE ========================

	uint resolution = 600;
	float viewX = 3.0, viewY = 35.0;
	double range = 60.0, degree = 1.8;

	ScanSonar sonar;
	sonar.setNumberOfBins(500);
	NormalDepthMap normal_depth_map(range);
	ImageViewerCaptureTool capture(640,480);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	osg::ref_ptr<osg::Group> root = new osg::Group();
//	makeSampleScene(root);
	makeManifold(root);

	normal_depth_map.addNodeChild(root);

	osgViewer::Viewer viewer;

	osg::ref_ptr<osgOcean::OceanScene> _oceanScene;


	// ======================= CORRECT SCENE =======================

//	osg::Matrix m = capture.getViewMatrix();
//	m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(90.0), osg::X_AXIS)));
////	m.preMult(osg::Matrix::translate(500.0f, 0.0f, 0.0f));
//
//
//	capture.setViewMatrix(m);

	// ======================= GRAB CAPTURE =======================


//	bool loop = true;
//	while(loop)
//	{
//
//		osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normal_depth_map.getNormalDepthMapNode());
//
//
//
//		// ======================= SONAR SIMULATION =======================
//
//		cv::Mat3f cvImage = convertShaderOSG2CV(osgImage);
//		cv::Mat raw_intensity = sonar.decodeShaderImage(cvImage);
//
//
//		// ======================= ROTATE THE CAMERA =======================
//
//		cv::imshow("teste", cvImage);
//
//		char k = cv::waitKey(0);
//
//		switch(k)
//		{
//			case 27: // ESC
//				loop = false;
//				break;
//
//			case 82: // UP
//				std::cout << "UP Pressed! +X" << std::endl;
//				m.preMult(osg::Matrix::translate(10.0f, 0.0f, 0.0f));
////				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(15.0), osg::X_AXIS)));
//				break;
//
//			case 84: // DOWN
//				std::cout << "DOWN Pressed! -X" << std::endl;
//				m.preMult(osg::Matrix::translate(-10.0f, 0.0f, 0.0f));
////				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-15.0), osg::X_AXIS)));
//				break;
//
//			case 83: // RIGHT
//				std::cout << "RIGHT Pressed! +Y" << std::endl;
//				m.preMult(osg::Matrix::translate(0.0f, 10.0f, 0.0f));
////				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(15.0), osg::Y_AXIS)));
//				break;
//
//			case 81: // LEFT
//				std::cout << "LEFT Pressed! -Y" << std::endl;
//				m.preMult(osg::Matrix::translate(0.0f,-10.0f, 0.0f));
////				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-15.0), osg::Y_AXIS)));
//				break;
//
//			case 85: // PAGE_UP
//				std::cout << "PAGE_UP Pressed! +Z" << std::endl;
//				m.preMult(osg::Matrix::translate(0.0f, 0.0f, 10.0f));
////				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(15.0), osg::Z_AXIS)));
//				break;
//
//			case 86: // PAGE_DOWN
//				std::cout << "PAGE_DOWN Pressed! -Z" << std::endl;
//				m.preMult(osg::Matrix::translate(0.0f, 0.0f, -10.0f));
////				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-15.0), osg::Z_AXIS)));
//				break;
//
//			default:
//				std::cout << "Rotate!" << std::endl;
//				m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-degree), osg::Z_AXIS)));
//				break;
//		}
//
//		capture.setViewMatrix(m);
//
//
//
//
//
//	}

	viewer.setSceneData(normal_depth_map.getNormalDepthMapNode());
	viewer.setUpViewInWindow( 150,150,800,600, 0 );
	viewer.run();





}

BOOST_AUTO_TEST_SUITE_END();
