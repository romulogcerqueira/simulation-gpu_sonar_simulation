// C++ includes
#include <iostream>

// Rock includes
#define BOOST_TEST_MODULE "Attenuation_test"
#include <gpu_sonar_simulation/Utils.hpp>
#include <boost/test/unit_test.hpp>

using namespace gpu_sonar_simulation;

BOOST_AUTO_TEST_SUITE(AcousticAttenuation)

BOOST_AUTO_TEST_CASE(attenuationCalculation_testCase){
    double frequency = 700.0;   // kHz
    double temperature = 20.0;  // celsius degrees
    double depth = 1;           // meters
    double salinity = 35;       // ppt
    double acidity = 8.1;       // pH

    double attenuationCoeff = underwaterSignalAttenuation(frequency, temperature, depth, salinity, acidity);

    BOOST_CHECK_CLOSE(attenuationCoeff, 0.0247, 3);
}

BOOST_AUTO_TEST_SUITE_END();
