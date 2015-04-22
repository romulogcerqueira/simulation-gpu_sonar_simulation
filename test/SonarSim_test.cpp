/*
 * MicronSim_test.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: romulo
 */

#include <gpu_sonar_simulation/SonarSim.hpp>
#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BOOST_TEST_MODULE "SonarSim_test"
#include <boost/test/unit_test.hpp>

using namespace gpu_sonar_simulation;

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



BOOST_AUTO_TEST_CASE(first_test) {

	using namespace gpu_sonar_simulation_MicronSim;

	int rows = 25, cols = 25;

	SonarSim sonar;

	// create a random image
	cv::Mat raw_image = createRandomImage(rows, cols);

	// print depth and normal matrixes
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			printf("D[%d][%d] = %.3f\n", i, j, raw_image.at<Vec3f>(i, j)[0]);
	printf("\n");
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			printf("N[%d][%d] = %.3f\n", i, j, raw_image.at<Vec3f>(i, j)[1]);

	cv::Mat raw_intensity = sonar.decodeRawImage(raw_image);
	std::vector<uint8_t> beam = sonar.getPingIntensity(raw_intensity);

	// print raw intensity
	std::cout << "Raw Intensity 3: " << raw_intensity << std::endl;

	// print beam data
	printf("\nBeam Data:\n");
	for (int i = 0; i < beam.size(); i++)
		printf("beam[%d] = %d\n", i, beam[i]);
}

BOOST_AUTO_TEST_SUITE_END();
