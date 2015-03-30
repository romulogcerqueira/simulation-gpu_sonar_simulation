#include <boost/test/unit_test.hpp>
#include <gpu_sonar_simulation/Dummy.hpp>

using namespace gpu_sonar_simulation;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    gpu_sonar_simulation::DummyClass dummy;
    dummy.welcome();
}
