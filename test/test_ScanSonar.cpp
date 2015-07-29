#include <gpu_sonar_simulation/ScanSonar.hpp>
#include <gpu_sonar_simulation/SonarUtils.hpp>
#include <vizkit3d_normal_depth_map/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/ImageViewerCaptureTool.hpp>

#include <iostream>
#include <stdio.h>
#include <cstdlib>

#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

#define BOOST_TEST_MODULE "ScanSonar_test"
#include <boost/test/unit_test.hpp>

using namespace gpu_sonar_simulation;
using namespace vizkit3d_normal_depth_map;

BOOST_AUTO_TEST_SUITE(gpu_sonar_simulation_MicronSim)

#define myrand ((float)(random())/(float)(RAND_MAX) )

// get the value of an environment variable
std::string getEnvVar(std::string env_name) {
	const char* env_p = std::getenv(env_name.c_str());
	std::string result(env_p);
	return result;
}

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

	std::string rock_path = getEnvVar("AUTOPROJ_CURRENT_ROOT");
	osg::Node* oilring = osgDB::readNodeFile(rock_path + "/simulation/gpu_sonar_simulation/test_data/oil_rig_manifold/visual.dae");

	osg::Matrix mtransf;
	mtransf.preMult(osg::Matrix::translate(0, 13, 0));
	mtransf.preMult(osg::Matrix::scale(0.1f,0.1f,0.1f));
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

	// init scene
	uint resolution = 600;
	float viewX = 3.0, viewY = 35.0;
	double range = 60.0, degree = 1.8;

	ScanSonar sonar;
	sonar.setNumberOfBins(500);
	NormalDepthMap normal_depth_map(range);
	ImageViewerCaptureTool capture(640,480);
	capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));

	osg::ref_ptr<osg::Group> root = new osg::Group();
	addOilRig(root);
	normal_depth_map.addNodeChild(root);

	double rot = 0.0;
	double transX = 0.0;
	double transY = 0.0;
	double transZ = 0.0;

	while(true)
	{
		// transformation matrixes
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


		// grab capture
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

		std::cout << "--- camera params ---" << std::endl;
		std::cout << "trans  : " << transX << "," << transY << "," << transZ << std::endl;
		std::cout << "eye    : " << eye.x() << "," << eye.y() << "," << eye.z() << std::endl;
		std::cout << "center : " << center.x() << "," << center.y() << "," << center.z() << std::endl;
		std::cout << "up     : " << up.x() << "," << up.y() << "," << up.z() << std::endl;
	}

//	osgViewer::Viewer viewer;
//	viewer.setUpViewInWindow(0,0,600,600);
//	viewer.setSceneData(normal_depth_map.getNormalDepthMapNode());
//	viewer.run();
}

BOOST_AUTO_TEST_SUITE_END();
