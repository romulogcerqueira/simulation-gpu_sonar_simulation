rock_library(gpu_sonar_simulation
    SOURCES ScanSonar.cpp SonarUtils.cpp MultibeamSonar.cpp SimSonar.cpp
    HEADERS ScanSonar.hpp SonarUtils.hpp MultibeamSonar.hpp SimSonar.hpp
    DEPS_PKGCONFIG opencv base-types openscenegraph)