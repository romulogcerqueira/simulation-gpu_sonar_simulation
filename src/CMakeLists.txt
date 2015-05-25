rock_library(gpu_sonar_simulation
    SOURCES ScanSonar.cpp SonarUtils.cpp
    HEADERS ScanSonar.hpp SonarUtils.hpp
    DEPS_PKGCONFIG opencv base-types openscenegraph)