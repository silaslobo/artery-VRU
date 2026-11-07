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

# libcarla and its headers require C++17, while Artery itself is built with
# C++11/14.  Keep the newer standard local to this target.
set_target_properties(carla PROPERTIES
    OUTPUT_NAME artery_carla
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    OMNETPP_LIBRARY ON
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})

# CarlaAvatar implements the traci::VehicleSink interface; the bundled TraCI
# client headers (src/ + src/traci/sumo) are needed at compile time, the
# symbols come from libtraci.so at load time.
target_include_directories(carla PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src/traci/sumo
    $<TARGET_PROPERTY:core,INCLUDE_DIRECTORIES>)
target_link_libraries(carla PUBLIC traci)

find_package(Boost REQUIRED COMPONENTS date_time system filesystem)
find_package(Threads REQUIRED)

# CARLA's static libraries have to be listed before their dependencies
target_link_libraries(carla PUBLIC
    Carla::Client
    ${CARLA_RECAST_LIBRARIES}
    Carla::Rpc
    Boost::date_time
    Boost::system
    Boost::filesystem
    Threads::Threads)

find_package(ZLIB QUIET)
if(TARGET ZLIB::ZLIB)
    target_link_libraries(carla PUBLIC ZLIB::ZLIB)
endif()

# CarlaImageSink writes PNGs through libpng's simplified API.  libpng is
# already a hard requirement of the CARLA client build (CarlaClient.cmake runs
# find_package(PNG REQUIRED)), so this adds no new dependency.
find_package(PNG REQUIRED)
target_link_libraries(carla PUBLIC PNG::PNG)

# unit test for the coordinate transform (plain C++ executable, no OMNeT++)
add_executable(carla_coordtest tests/CoordTransformTest.cc)
set_target_properties(carla_coordtest PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_include_directories(carla_coordtest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
add_test(NAME coordtransform COMMAND carla_coordtest)

install(TARGETS carla LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_DATADIR}/ned/carla FILES_MATCHING PATTERN "*.ned")
set_property(TARGET carla APPEND PROPERTY INSTALL_NED_FOLDERS ${CMAKE_INSTALL_DATADIR}/ned/carla)
