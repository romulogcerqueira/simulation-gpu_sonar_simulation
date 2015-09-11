rock_library(gpu_sonar_simulation
    SOURCES ScanSonar.cpp Utils.cpp MultibeamSonar.cpp CommonSonar.cpp
    HEADERS ScanSonar.hpp Utils.hpp MultibeamSonar.hpp CommonSonar.hpp
    DEPS_PKGCONFIG opencv base-types openscenegraph)