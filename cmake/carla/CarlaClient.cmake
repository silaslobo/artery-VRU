# Builds the CARLA 1.0 C++ client library (LibCarla) from the pinned
# submodule and collects it, together with everything it needs at link
# time, into a self-contained prefix under build/extern/carla/install.
#
# CARLA 1.0 rewrote its build system: the top-level CMakeLists.txt +
# CMake/Dependencies.cmake fetch all dependencies (Boost 1.90, Eigen, zlib,
# libpng, rpclib, recastnavigation) with FetchContent, and LibCarla has a
# normal CMakeLists.txt — no more `make setup` / LibCarla/cmake trick, no
# more -DBOOST_INCLUDE_PATH-style cache plumbing, and no local source
# patches at all.
#
# Dependency versions are exactly what upstream CARLA pins (FetchContent,
# branch archives of the carla-simulator forks of rpclib/recastnavigation).
# We deliberately do NOT re-pin them locally: CARLA owns compatibility,
# and one more version voice would only add drift.  Requires python3-dev
# at configure time (BUILD_PYTHON_API is kept ON, see below).
#
# Produces exactly one IMPORTED target (consumed by src/carla/CMakeLists.txt):
#
#   Carla::Client  - libcarla-client.a, its headers (carla/, boost/, rpc/,
#                    Recast, png - all merged into one prefix by the sysroot
#                    script), and every vendored static archive it needs
#                    (Recast/Detour*, rpclib, Boost, png, zlib) as interface
#                    dependencies in correct link order.
#
# Nothing else is exported; consumption is
#   target_link_libraries(x PRIVATE Carla::Client Threads::Threads)
#
# Note: CARLA 1.0's public API uses std::shared_ptr (carla::SharedPtr),
# C++20, and rpc::EpisodeSettings with std::optional — see src/carla.

include(ExternalProject)

set(CARLA_SRC ${PROJECT_SOURCE_DIR}/extern/carla)
set(CARLA_BUILD_DIR ${CMAKE_BINARY_DIR}/extern/carla)
set(CARLA_INSTALL_DIR ${CARLA_BUILD_DIR}/install)
# The IMPORTED targets below reference this prefix before the ExternalProject
# has populated it; CMake rejects non-existent INTERFACE_INCLUDE_DIRECTORIES
# at configure time unless we create the directories here.
file(MAKE_DIRECTORY ${CARLA_INSTALL_DIR}/include ${CARLA_INSTALL_DIR}/lib)

# BUILD_PYTHON_API stays at upstream's default (ON) on purpose.  Turning it
# off hits a configure-time contradiction in CMake/Dependencies.cmake
# (`python` is listed in BOOST_INCLUDED_PROJECTS unconditionally while
# BOOST_ENABLE_PYTHON follows BUILD_PYTHON_API -> "Boost library 'python'
# has been explicitly requested, but BOOST_ENABLE_PYTHON is OFF"), and both
# variables are re-imposed on every configure, so no -D override helps.
# Building with python enabled needs python3-dev, costs some Boost compile
# time, and keeps the submodule pristine (no PATCH_COMMAND, no '-dirty'
# version suffix).  The wheel target (carla-python-api) is *not* built here.

# The Boost zip extraction (and thus configure) fails under a non-UTF-8
# locale ("Pathname cannot be converted from UTF-8 to current locale"), so
# force C.UTF-8 for the sub-build regardless of the caller's environment.
set(CARLA_ENV env LC_ALL=C.UTF-8 LANG=C.UTF-8)

ExternalProject_Add(carla_client
    SOURCE_DIR ${CARLA_SRC}
    BINARY_DIR ${CARLA_BUILD_DIR}
    PREFIX ${CARLA_BUILD_DIR}/prefix
    CONFIGURE_COMMAND ${CARLA_ENV} ${CMAKE_COMMAND} -S ${CARLA_SRC} -B ${CARLA_BUILD_DIR}
        -DCMAKE_BUILD_TYPE=Release
        # Position independent: Artery loads this static library into
        # OMNeT++ shared libraries.
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_CXX_FLAGS=-fPIC
        -DBUILD_CARLA_CLIENT=ON
        -DBUILD_CARLA_SERVER=OFF
        -DBUILD_CARLA_UNREAL=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_LIBCARLA_TESTS=OFF
        -DBUILD_PYTHON_API=ON
        -DBUILD_OSM_WORLD_RENDERER=OFF
        -DENABLE_ROS2=OFF
        -DCMAKE_INSTALL_PREFIX=${CARLA_INSTALL_DIR}
    BUILD_COMMAND ${CARLA_ENV} ${CMAKE_COMMAND} --build ${CARLA_BUILD_DIR} --target carla-client
    INSTALL_COMMAND ${CARLA_ENV} ${CMAKE_COMMAND} --install ${CARLA_BUILD_DIR}
    && ${CARLA_ENV} bash ${PROJECT_SOURCE_DIR}/cmake/carla/carla-sysroot-install.sh
             ${CARLA_BUILD_DIR} ${CARLA_INSTALL_DIR}
)

# --------------------------------------------------------------------------
# Imported targets
# --------------------------------------------------------------------------
#
# The vendored archives in INTERFACE_LINK_LIBRARIES are ORDERED and must
# stay so: these are static libraries, the linker resolves left-to-right,
# and the archives carry no dependency info between each other -
#   libcarla-client.a  ->  Recast/DetourCrowd/Detour (road navmesh;
#                         1.0 no longer needs DetourTileCache)
#                   ->  librpc (msgpack/RPC transport)
#                   ->  Boost (date_time for carla::time_duration, filesystem
#                         for GIL, graph/random/serialization for client-side
#                         maps, thread/chrono/atomic/context underneath;
#                         asio is header-only)
#                   ->  png/z (image decoding, rpclib payloads; last so the
#                         .so stays self-contained without a system libpng)
# CMake appends the interface dependencies of an IMPORTED STATIC target
# directly after the target itself and preserves the relative order of
# entries without a dependency relation, so this list reaches the link line
# verbatim.  Wrapping the same list in an INTERFACE library does NOT work:
# CMake hoists static entries of INTERFACE libraries to the end of the link
# line, where they no longer satisfy libcarla's references (measured).
set(CARLA_LIB ${CARLA_INSTALL_DIR}/lib)
# (plain file-local list; the semicolon-separated entries must not pick up
# newlines, hence no quoted multi-line string)
set(CARLA_CLIENT_DEPS
    ${CARLA_LIB}/libRecast.a
    ${CARLA_LIB}/libDetourCrowd.a
    ${CARLA_LIB}/libDetour.a
    ${CARLA_LIB}/librpc.a
    ${CARLA_LIB}/libboost_date_time.a
    ${CARLA_LIB}/libboost_filesystem.a
    ${CARLA_LIB}/libboost_container.a
    ${CARLA_LIB}/libboost_chrono.a
    ${CARLA_LIB}/libboost_thread.a
    ${CARLA_LIB}/libboost_atomic.a
    ${CARLA_LIB}/libboost_context.a
    ${CARLA_LIB}/libboost_serialization.a
    ${CARLA_LIB}/libboost_random.a
    ${CARLA_LIB}/libboost_graph.a
    ${CARLA_LIB}/libpng.a
    ${CARLA_LIB}/libz.a)
add_library(Carla::Client STATIC IMPORTED GLOBAL)
set_target_properties(Carla::Client PROPERTIES
    IMPORTED_LOCATION ${CARLA_LIB}/libcarla-client.a
    # Includes carla/ (installed by CARLA) and boost/, rpc/, Recast, png
    # (merged by the sysroot script) - src/carla must compile against the
    # very headers libcarla was built with, not the system ones.
    INTERFACE_INCLUDE_DIRECTORIES ${CARLA_INSTALL_DIR}/include
    # Mirrors CARLA's CARLA_COMMON_DEFINITIONS for the default
    # LIBCARLA_IMAGE_SUPPORTED_FORMATS="png" - the carla::image types are
    # compile-gated on it, so consumers need it too.
    INTERFACE_COMPILE_DEFINITIONS LIBCARLA_IMAGE_SUPPORT_PNG=1
    INTERFACE_COMPILE_FEATURES cxx_std_20
    INTERFACE_LINK_LIBRARIES "${CARLA_CLIENT_DEPS}"
)
add_dependencies(Carla::Client carla_client)
