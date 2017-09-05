include_directories(${CMAKE_SOURCE_DIR})

add_library(offline_slam SHARED
  offline_slam.cpp
  offline_slam.h
  )

target_link_libraries(offline_slam
  argos3core_simulator
  argos3plugin_simulator_kheperaiv)
