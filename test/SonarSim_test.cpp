/*
 * MicronSim_test.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: romulo
 */

#include <gpu_sonar_simulation/ScanningSonar.hpp>
#include <gpu_sonar_simulation/SonarConfig.hpp>
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

    osg::ShapeDrawable *refSph1 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-10,0,0), 1));
	refSph1->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));

	osg::ShapeDrawable *refSph2 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10,0,0), 1));
	refSph2->setColor(osg::Vec4(1.0f,0.5f,0.0f,1.0f));

    osg::ShapeDrawable *refBox1 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 10), 1));
    refBox1->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));

	osg::ShapeDrawable *refBox2 = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, -10), 1));
	refBox2->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

    // Sphere
	object->addDrawable(refSph1);
    object->addDrawable(refSph2);
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,10,0), 1)));
    object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-10,0), 1)));
    object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-10,-10,0), 1)));
    object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-10,10,0), 1)));
    object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10,10,0), 1)));
    object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(10,-10,0), 1)));

    // Box
    object->addDrawable(refBox1);
	object->addDrawable(refBox2);
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(10, 0, 10), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(10, 0, -10), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(-10, 0, 10), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(-10, 0, -10), 1)));

	// Cone
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 10, 10), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 10, -10), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, -10, 10), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, -10, -10), 1, 1)));

	root->addChild(object);
}


BOOST_AUTO_TEST_CASE(drivers_integration) {

	using namespace gpu_sonar_simulation_MicronSim;

	SonarConfig config;
	ScanningSonar sonar;

	sonar.setup(config);

	cv::Mat raw_image = createRandomImage(640, 480);

	cv::Mat raw_intensity = sonar.decodeRawImage(raw_image);

	std::vector<uint8_t> data = sonar.getPingIntensity(raw_intensity);

	base::samples::SonarBeam sonar_beam = sonar.simulateSonarBeam(data);
}

BOOST_AUTO_TEST_CASE(complete_rotate_image) {

	osg::ref_ptr<osg::Image> osg_image;

	uint width = 640, height = 480;
	float range = 75.0;

	NormalDepthMap normal_depth_map(range);

	osgViewer::Viewer viewer;

	osg::ref_ptr<osg::Group> root = new osg::Group();
	makeSampleScene(root);

	root = normal_depth_map.applyShaderNormalDepthMap(root);

	viewer.setSceneData(root);
	viewer.setUpViewInWindow(0,0,width,height);


	viewer.realize();
	double d = 1;

	osg::Matrix m = viewer.getCamera()->getViewMatrix();
	m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians( 90.0),osg::X_AXIS)));


	while(!viewer.done())
	{
		m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-d),osg::Z_AXIS)));
		viewer.getCamera()->setViewMatrix(m);
		viewer.frame();
	}
}


BOOST_AUTO_TEST_CASE(complete_rotate_image_with_capture) {

	osg::ref_ptr<osg::Image> osg_image;

	uint width = 640, height = 480;
	float range = 75.0;

	NormalDepthMap normal_depth_map(range);

	osgViewer::Viewer viewer;

	osg::ref_ptr<osg::Group> root = new osg::Group();
	makeSampleScene(root);

	root = normal_depth_map.applyShaderNormalDepthMap(root);

	viewer.setSceneData(root);
	viewer.setUpViewInWindow(0,0,width,height);


	viewer.realize();
	double d = 1;

	osg::Matrix m = viewer.getCamera()->getViewMatrix();
	m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians( 90.0),osg::X_AXIS)));


	while(!viewer.done())
	{
		m.preMult(osg::Matrix::rotate(osg::Quat(osg::DegreesToRadians(-d),osg::Z_AXIS)));
		viewer.getCamera()->setViewMatrix(m);
		viewer.frame();
	}
}

BOOST_AUTO_TEST_SUITE_END();
