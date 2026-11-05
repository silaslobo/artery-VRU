# CARLA co-simulation support for Artery.
#
# Provides the Carla::Client backed OMNeT++ modules (see CarlaScenarioManager).
# The CARLA C++ client itself is built by cmake/carla/CarlaClient.cmake, which
# is included from the top-level CMakeLists.txt.

set(SOURCES
    CarlaScenarioManager.cc
    CarlaSensorReading.cc
    CarlaSensorSink.cc
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

target_include_directories(carla PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<TARGET_PROPERTY:core,INCLUDE_DIRECTORIES>)

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

install(TARGETS carla LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_DATADIR}/ned/carla FILES_MATCHING PATTERN "*.ned")
set_property(TARGET carla APPEND PROPERTY INSTALL_NED_FOLDERS ${CMAKE_INSTALL_DATADIR}/ned/carla)
