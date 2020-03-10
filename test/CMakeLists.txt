rock_testsuite(Sonar_core Sonar_test.cpp
	DEPS gpu_sonar_simulation
	LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv normal_depth_map)

rock_testsuite(Attenuation_core Attenuation_test.cpp
	DEPS gpu_sonar_simulation
	LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv normal_depth_map)
