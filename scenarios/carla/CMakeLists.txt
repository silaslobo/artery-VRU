# CARLA co-simulation scenario.
#
# Combined SUMO + CARLA scenario (omnetpp-sumo.ini, network carla.CarlaSumoWorld):
#   cmake --build build --target run_carla_sumo_scenario
# The generated run script loads core, traci and carla automatically
# add_opp_run() only walks the dependency graph of non-shared targets, so an
# INTERFACE aggregator is used to make the run script load libartery_carla.so
# *and* libtraci.so (plus their NED folders; ${CMAKE_CURRENT_SOURCE_DIR}
# provides the carla.* networks of this scenario).
add_library(carla_sumo_deps INTERFACE)
target_link_libraries(carla_sumo_deps INTERFACE carla traci)
add_opp_run(carla_sumo_scenario CONFIG omnetpp-sumo.ini DEPENDENCY carla_sumo_deps
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# ---------------------------------------------------------------------------
# Map artifacts (scenarios/carla/maps)
#
#   gen_xodr.py --> intersection.xodr (+ .spawn / .lanepoints sidecars)
#   netconvert  --> intersection.net.xml   (normalization MUST stay disabled)
#   gen_routes.py --> intersection.rou.xml
#
# The generated files are committed (regeneration is deterministic); this
# target only refreshes them when the generator changes, so the two
# simulators can never silently drift apart.
# ---------------------------------------------------------------------------
find_program(PYTHON3_EXECUTABLE NAMES python3)
find_program(NETCONVERT_EXECUTABLE NAMES netconvert)

set(CARLA_MAP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/maps)
set(CARLA_MAP_GEN ${CMAKE_SOURCE_DIR}/tools/carla_map)

if(PYTHON3_EXECUTABLE AND NETCONVERT_EXECUTABLE)
    add_custom_command(
        OUTPUT ${CARLA_MAP_DIR}/intersection.xodr
               ${CARLA_MAP_DIR}/intersection.xodr.spawn
               ${CARLA_MAP_DIR}/intersection.xodr.lanepoints
        COMMAND ${PYTHON3_EXECUTABLE} ${CARLA_MAP_GEN}/gen_xodr.py
                --out ${CARLA_MAP_DIR}/intersection.xodr --verify
        DEPENDS ${CARLA_MAP_GEN}/gen_xodr.py
        COMMENT "Generating intersection.xodr (gen_xodr.py)")

    add_custom_command(
        OUTPUT ${CARLA_MAP_DIR}/intersection.net.xml
        COMMAND ${NETCONVERT_EXECUTABLE}
                --opendrive-files ${CARLA_MAP_DIR}/intersection.xodr
                --offset.disable-normalization=true
                --tls.guess-signals --tls.guess-signals.dist=15
                --opendrive.import-all-lanes
                --output-file ${CARLA_MAP_DIR}/intersection.net.xml
        DEPENDS ${CARLA_MAP_DIR}/intersection.xodr
        COMMENT "Converting intersection.xodr -> intersection.net.xml")

    add_custom_command(
        OUTPUT ${CARLA_MAP_DIR}/intersection.rou.xml
        COMMAND ${PYTHON3_EXECUTABLE} ${CARLA_MAP_GEN}/gen_routes.py
                --xodr ${CARLA_MAP_DIR}/intersection.xodr
                --net ${CARLA_MAP_DIR}/intersection.net.xml
                --only=-10:-15
                --out ${CARLA_MAP_DIR}/intersection.rou.xml
        DEPENDS ${CARLA_MAP_DIR}/intersection.net.xml
                ${CARLA_MAP_DIR}/intersection.xodr
                ${CARLA_MAP_GEN}/gen_routes.py
        COMMENT "Generating intersection.rou.xml (gen_routes.py)")

    add_custom_target(carla_maps ALL
        DEPENDS ${CARLA_MAP_DIR}/intersection.rou.xml)
else()
    message(WARNING "python3 and/or netconvert not found: cannot regenerate the "
                    "map artifacts of scenarios/carla (run 'carla_maps' on a "
                    "machine that has them; the outputs are gitignored)")
endif()
