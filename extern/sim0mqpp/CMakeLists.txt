cmake_minimum_required(VERSION 3.6)
project(sim0mqpp VERSION 0.1 LANGUAGES CXX)
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

find_package(Boost 1.58 REQUIRED)

set(SOURCES
    src/any.cpp
    src/buffer_serialization.cpp
    src/counter_serialization.cpp
    src/identifier.cpp
    src/message.cpp
    src/primitive.cpp
    src/quantity.cpp
    src/string.cpp
    src/unit.cpp
)

if(NOT DEFINED SIM0MQPP_SHARED_LIB)
    set(SIM0MQPP_SHARED_LIB ${BUILD_SHARED_LIBS})
endif()
option(SIM0MQPP_SHARED_LIB "Build sim0mq++ as shared library" ${SIM0MQPP_SHARED_LIB})
if(SIM0MQPP_SHARED_LIB)
    add_library(sim0mqpp SHARED ${SOURCES})
else()
    add_library(sim0mqpp STATIC ${SOURCES})
endif()

target_include_directories(sim0mqpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
)
set_target_properties(sim0mqpp PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# install and export
add_library(sim0mqpp::sim0mqpp ALIAS sim0mqpp)
option(SIM0MQPP_INSTALL "Generate install target for sim0mq++" ON)
if(SIM0MQPP_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(sim0mqpp-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )
    install(TARGETS sim0mqpp EXPORT sim0mqpp-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(EXPORT sim0mqpp-targets
        FILE sim0mqpp-targets.cmake
        NAMESPACE sim0mqpp::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sim0mqpp
    )
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    configure_file(sim0mqpp-config.cmake.in sim0mqpp-config.cmake @ONLY)
    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/sim0mqpp-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/sim0mqpp-version.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sim0mqpp
    )

    set(CMAKE_EXPORT_PACKAGE_REGISTRY OFF)
    export(TARGETS sim0mqpp NAMESPACE sim0mqpp:: FILE sim0mqpp-targets.cmake)
    export(PACKAGE sim0mqpp)
endif()
