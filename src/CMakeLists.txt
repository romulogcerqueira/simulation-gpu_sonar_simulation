rock_library(gpu_sonar_simulation
    SOURCES Sonar.cpp Utils.cpp
    HEADERS Sonar.hpp Utils.hpp
    DEPS_PKGCONFIG base-types openscenegraph opencv)
