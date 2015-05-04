rock_library(gpu_sonar_simulation
    SOURCES SonarSim.cpp Utils.cpp
    HEADERS SonarSim.hpp Utils.hpp
    DEPS_PKGCONFIG opencv base-types openscenegraph)