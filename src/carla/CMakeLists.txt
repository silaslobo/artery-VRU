# CARLA co-simulation support for Artery.
#
# Provides the Carla::Client backed OMNeT++ modules (see CarlaScenarioManager).
# The CARLA C++ client itself is built by cmake/carla/CarlaClient.cmake, which
# is included from the top-level CMakeLists.txt.

set(SOURCES
    CarlaScenarioManager.cc
    CarlaAvatar.cc
    CarlaSensorReading.cc
    CarlaSensorSink.cc
    CarlaImageSink.cc
)

add_library(carla SHARED ${SOURCES})

# libcarla (CARLA 1.0) requires C++20, while Artery itself is built with
# C++11/14.  Keep the newer standard local to this target.
set_target_properties(carla PROPERTIES
    OUTPUT_NAME artery_carla
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    OMNETPP_LIBRARY ON
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})

# CarlaAvatar implements the traci::VehicleSink interface; the bundled TraCI
# client headers (src/ + src/traci/sumo) are needed at compile time, the
# symbols come from libtraci.so at load time.
# Everything below is PRIVATE on purpose: this library must export no usage
# requirements.  artery core links `carla` PUBLIC (so add_opp_run finds the
# module), but core is built against the system Boost at C++14 — leaking the
# CARLA 1.0 vendored Boost tree or C++20 into it breaks its compilation
# (measured).  No consumer compiles against carla/ headers anyway.
target_include_directories(carla PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src/traci/sumo
    $<TARGET_PROPERTY:core,INCLUDE_DIRECTORIES>)
target_link_libraries(carla PRIVATE traci)

find_package(Threads REQUIRED)

# CARLA 1.0 vendors its dependencies (Boost 1.90, rpclib, Recast, libpng,
# zlib) as static archives built by cmake/carla/CarlaClient.cmake.  They are
# linked instead of the system libraries so libcarla always sees the exact
# versions it was compiled against; the system Boost (used by artery core)
# is kept away from them by --exclude-libs below.  Carla::Client carries the
# full ordered archive list as interface dependencies (see CarlaClient.cmake),
# so linking the single target is everything this library needs.
target_link_libraries(carla PRIVATE Carla::Client Threads::Threads)

# Hide the vendored (Boost 1.90 & friends) archive symbols inside this
# shared library: artery core is linked against the system Boost, and two
# Boost versions exported from one process would interbind at runtime.
# Our own objects (the OMNeT++ module symbols) stay exported.
target_link_options(carla PRIVATE "LINKER:--exclude-libs,ALL")

# unit test for the coordinate transform (plain C++ executable, no OMNeT++)
add_executable(carla_coordtest tests/CoordTransformTest.cc)
set_target_properties(carla_coordtest PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_include_directories(carla_coordtest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
add_test(NAME coordtransform COMMAND carla_coordtest)

install(TARGETS carla LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_DATADIR}/ned/carla FILES_MATCHING PATTERN "*.ned")
set_property(TARGET carla APPEND PROPERTY INSTALL_NED_FOLDERS ${CMAKE_INSTALL_DATADIR}/ned/carla)
