# CARLA co-simulation example.
#
# Runs the CarlaWorld network against a CARLA server:
#   cmake --build build --target run_carla_scenario
add_opp_run(carla_scenario WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
