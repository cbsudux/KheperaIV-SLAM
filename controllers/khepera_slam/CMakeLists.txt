include_directories(${CMAKE_SOURCE_DIR})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/BreezySLAM/c")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/BreezySLAM/cpp")

add_library(khepera_slam MODULE
  kheperaiv_slam.cpp
  kheperaiv_slam.h
#  BreezySLAM/c/coreslam_armv7l.c
  BreezySLAM/c/coreslam.c
  BreezySLAM/c/coreslam.h
#  BreezySLAM/c/coreslam_i686.c
  BreezySLAM/c/coreslam_internals.h
  BreezySLAM/c/coreslam_sisd.c
  BreezySLAM/c/list.txt
  BreezySLAM/c/random.c
  BreezySLAM/c/random.h
  BreezySLAM/c/ziggurat.c
  BreezySLAM/c/ziggurat.h
  BreezySLAM/cpp/algorithms.cpp
  BreezySLAM/cpp/algorithms.hpp
  BreezySLAM/cpp/BreezySLAM.hpp
  BreezySLAM/cpp/Laser.cpp
  BreezySLAM/cpp/Laser.hpp
  BreezySLAM/cpp/Map.cpp
  BreezySLAM/cpp/Map.hpp
  BreezySLAM/cpp/Position.hpp
  BreezySLAM/cpp/Scan.cpp
  BreezySLAM/cpp/Scan.hpp
  BreezySLAM/cpp/Velocities.hpp
  BreezySLAM/cpp/WheeledRobot.cpp
  BreezySLAM/cpp/WheeledRobot.hpp

  )

target_link_libraries(khepera_slam
  argos3core_simulator
  argos3plugin_simulator_kheperaiv)
