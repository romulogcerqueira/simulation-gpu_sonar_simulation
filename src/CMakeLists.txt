rock_library(gpu_sonar_simulation
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(gpu_sonar_simulation_bin Main.cpp
    DEPS gpu_sonar_simulation)

