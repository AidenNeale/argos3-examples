add_library(pipuck_foraging MODULE
  pipuck_foraging.h
  pipuck_foraging.cpp)
target_link_libraries(pipuck_foraging
  argos3core_simulator
  argos3plugin_simulator_pipuck)
