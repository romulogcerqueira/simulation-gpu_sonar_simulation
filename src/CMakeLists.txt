rock_library(gpu_sonar_simulation
    SOURCES ScanningSonar.cpp SonarUtils.cpp
    HEADERS ScanningSonar.hpp SonarUtils.hpp SonarConfig.hpp
    DEPS_PKGCONFIG opencv base-types openscenegraph)