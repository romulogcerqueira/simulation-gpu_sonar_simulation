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
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

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

	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,-20,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3( 40,0,0), 1, 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3( 0,60,0), 1)));
	object->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(-80,0,0), 1, 1)));


	root->addChild(object);
}

//draw the scene with a small ball in the center with a big cube, Box and cone in back
void addManifold(osg::ref_ptr<osg::Group> root) {

	osg::Node* manifold = osgDB::readNodeFile("/home/romulogc/flatfish_meshs/simple_manifold/model.dae");

	osg::Matrix mtransf;
	mtransf.preMult(osg::Matrix::translate(10, 0, -2.0));
	mtransf.preMult(osg::Matrix::scale(0.01f,0.01f,0.01f));
	mtransf.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));
	osg::MatrixTransform *ptransform = new osg::MatrixTransform();
	ptransform->setMatrix(mtransf);
	ptransform->addChild(manifold);

	root->addChild(ptransform);
}

//draw the scene with a small ball in the center with a big cube, Box and cone in back
void addFlatfish(osg::ref_ptr<osg::Group> root) {

	osg::Node* flatfish = osgDB::readNodeFile("/home/romulogc/rock_robotics/simulation/orogen/gpu_sonar_simulation/resources/flatfish_02.dae");

	osg::Matrix mtransf;
	mtransf.preMult(osg::Matrix::translate(-15, -5, 2.5));
	mtransf.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));
	osg::MatrixTransform *ptransform = new osg::MatrixTransform();
	ptransform->setMatrix(mtransf);
	ptransform->addChild(flatfish);

	root->addChild(ptransform);
}

void addOilRing(osg::ref_ptr<osg::Group> root){

	osg::Node* oilring = osgDB::readNodeFile("/home/romulogc/rock_robotics/simulation/orogen/gpu_sonar_simulation/resources/oil_rig_manifold.dae");

	osg::Matrix mtransf;
	mtransf.preMult(osg::Matrix::translate(0, 13, 0));
	mtransf.preMult(osg::Matrix::scale(0.1f,0.1f,0.1f));
	mtransf.preMult(osg::Matrix::rotate(-90, osg::Vec3(0,0,1)));
	osg::MatrixTransform *ptransform = new osg::MatrixTransform();
	ptransform->setMatrix(mtransf);
	ptransform->addChild(oilring);

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

	base::samples::SonarBeam sonar_beam = sonar.simulateSonarBeam(data);
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
	addManifold(root);
	addFlatfish(root);
	addOilRing(root);


	normal_depth_map.addNodeChild(root);

	bool loop = true;

	double rot = 0.0;

	double transX = 0.0;
	double transY = 0.0;
	double transZ = 0.0;

	while(loop)
	{

		std::cout << "camera params" << std::endl;


		// ================== TRANSFORMATION MATRIXES ==================


		osg::Matrixd matrix;
		matrix.setTrans(osg::Vec3(transX, transY, transZ));
		matrix.setRotate(osg::Quat(rot, osg::Vec3(0, 0, 1)));
		matrix.invert(matrix);

		osg::Matrixd rock_coordinate_system =
				osg::Matrixd::rotate(-M_PI_2, osg::Vec3(0, 0, -1)) *
				osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));


		osg::Matrixd m = matrix * rock_coordinate_system;


		osg::Vec3 eye, center, up;
		m.getLookAt(eye, center, up);
		capture.setCameraPosition(eye, center, up);

		// ======================= GRAB CAPTURE =======================

		osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normal_depth_map.getNormalDepthMapNode());
		cv::Mat3f cvImage = convertShaderOSG2CV(osgImage);


		cv::imshow("Normal Depth Map", cvImage);
		char k = cv::waitKey(0);


		switch(k)
		{

			case 27:		//	ESC
				exit(0);
				break;
			case 85:		// UP
				transZ += 2.0f;
				break;
			case 86:		// DOWN
				transZ -= 2.0f;
				break;
			case 81:		// LEFT
				transY += 2.0f;
				break;
			case 83:		// RIGHT
				transY -= 2.0f;
				break;
			case 82:		// PAGE UP
				transX += 2.0f;
				break;
			case 84:		// PAGE DOWN
				transX -= 2.0f;
				break;
			case 122:		// Z
				rot += osg::DegreesToRadians(5.0);
				break;
			case 120:		// X
				rot -= osg::DegreesToRadians(5.0);
				break;

		}

		printf("trans: %f %f %f\n", transX, transY, transZ);
		printf("eye: %f %f %f\n", eye.x(), eye.y(), eye.z());
		printf("center: %f %f %f\n", center.x(), center.y(), center.z());
		printf("up: %f %f %f\n", up.x(), up.y(), up.z());

	}


//	osgViewer::Viewer viewer;
//	viewer.setUpViewInWindow(0,0,600,600);
//	viewer.setSceneData(normal_depth_map.getNormalDepthMapNode());
//	viewer.run();
}





BOOST_AUTO_TEST_SUITE_END();
