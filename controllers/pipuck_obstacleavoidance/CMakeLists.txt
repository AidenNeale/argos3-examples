add_library(pipuck_obstacleavoidance MODULE
  pipuck_obstacleavoidance.h
  pipuck_obstacleavoidance.cpp)
target_link_libraries(pipuck_obstacleavoidance
  argos3core_simulator
  argos3plugin_simulator_pipuck)
