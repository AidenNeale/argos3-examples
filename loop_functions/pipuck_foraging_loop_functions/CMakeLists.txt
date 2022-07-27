# compile test loop functions
add_library(pipuck_foraging_loop_functions MODULE
  pipuck_foraging_loop_functions.h
  pipuck_foraging_loop_functions.cpp)
target_link_libraries(pipuck_foraging_loop_functions
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_pipuck)

