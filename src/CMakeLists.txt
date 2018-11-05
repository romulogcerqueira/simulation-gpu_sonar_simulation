rock_library(gpu_sonar_simulation
    SOURCES Sonar.cpp Utils.cpp SonarSimulation.cpp
    HEADERS Sonar.hpp Utils.hpp SonarSimulation.hpp
    DEPS_PKGCONFIG base-types openscenegraph opencv normal_depth_map
    frame_helper)

rock_executable(main main.cpp
    DEPS gpu_sonar_simulation)
