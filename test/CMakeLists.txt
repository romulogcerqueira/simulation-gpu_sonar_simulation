rock_testsuite(SonarSim_core SonarSim_test.cpp
	DEPS gpu_sonar_simulation
	LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv vizkit3d_normal_depth_map)