#ifndef _SONARCONFIG_HPP_
#define _SONARCONFIG_HPP_

#include <base/Angle.hpp>

namespace gpu_sonar_simulation{

class SonarConfig
{
public:
	base::Angle left_limit;
	base::Angle right_limit;
	base::Angle angular_resolution;

	double max_distance;
	double min_distance;
	double resolution;
	double speed_of_sound;

	uint8_t ad_low;
	uint8_t ad_span;

	SonarConfig():
		left_limit(base::Angle::fromRad(M_PI)),
		right_limit(base::Angle::fromRad(-M_PI)),
		angular_resolution(base::Angle::fromRad(5.0/180.0*M_PI)),
		max_distance(30.0),
		min_distance(1.0),
		resolution(0.1),
		speed_of_sound(1500.0),
		ad_low(29),
		ad_span(66)
	{};
};
}

#endif
