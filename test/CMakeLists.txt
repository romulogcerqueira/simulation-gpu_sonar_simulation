rock_testsuite(Sonar_core test_Sonar.cpp
	DEPS gpu_sonar_simulation
	LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv normal_depth_map)
