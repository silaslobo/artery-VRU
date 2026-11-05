# Builds the CARLA C++ client library from the pinned submodules
# (carla, rpclib, recastnavigation) using ExternalProject.
#
# Produces the following IMPORTED targets:
#   Carla::Client     - libcarla_client.a + carla headers
#   Carla::Rpc        - librpc.a
#   Carla::Recast     - libRecast.a, libDetour.a, libDetourCrowd.a, libDetourTileCache.a
#
# The CARLA client source is patched with carla-client-mutex.patch
# (adds missing <mutex> includes required when using libstdc++/system Boost).

include(ExternalProject)

# Each dependency builds and installs into its own directory under build/extern/.
set(CARLA_EXTERN_DIR ${CMAKE_CURRENT_BINARY_DIR}/extern)

find_package(Boost REQUIRED)
find_package(PNG REQUIRED)

# Boost and libpng are provided system-wide. The CARLA client build uses their
# headers at compile time (BOOST_INCLUDE_PATH / LIBPNG_INCLUDE_PATH) but, after
# our carla-client-cmake.patch, no longer installs their headers/libs. Their lib
# paths are unused in the client build, so we pass the system include dirs only.

# --------------------------------------------------------------------------
# rpclib
# --------------------------------------------------------------------------
set(RPCLIB_SRC ${PROJECT_SOURCE_DIR}/extern/rpclib)
set(RPCLIB_INSTALL ${CARLA_EXTERN_DIR}/rpclib/install)

ExternalProject_Add(carla_rpclib
    SOURCE_DIR ${RPCLIB_SRC}
    BINARY_DIR ${CMAKE_BINARY_DIR}/extern/rpclib
    PREFIX ${CMAKE_BINARY_DIR}/extern/rpclib/prefix
    BUILD_ALWAYS OFF
    INSTALL_DIR ${RPCLIB_INSTALL}
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DRPCLIB_BUILD_TESTS=OFF
        -DRPCLIB_BUILD_EXAMPLES=OFF
        -DCMAKE_CXX_FLAGS=-fPIC
        -DCMAKE_INSTALL_PREFIX=${RPCLIB_INSTALL}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install --config Release
)

# --------------------------------------------------------------------------
# Recast & Detour
# --------------------------------------------------------------------------
set(RECAST_SRC ${PROJECT_SOURCE_DIR}/extern/recastnavigation)
set(RECAST_INSTALL ${CARLA_EXTERN_DIR}/recastnavigation/install)

ExternalProject_Add(carla_recast
    SOURCE_DIR ${RECAST_SRC}
    BINARY_DIR ${CMAKE_BINARY_DIR}/extern/recastnavigation
    PREFIX ${CMAKE_BINARY_DIR}/extern/recastnavigation/prefix
    INSTALL_DIR ${RECAST_INSTALL}
    CMAKE_ARGS
        -DRECASTNAVIGATION_BUILDER=OFF
        -DRECASTNAVIGATION_DEMO=OFF
        -DRECASTNAVIGATION_TESTS=OFF
        -DRECASTNAVIGATION_EXAMPLES=OFF
        -DRECASTNAVIGATION_STATIC=ON
        -DCMAKE_CXX_FLAGS=-fPIC
        -DCMAKE_INSTALL_PREFIX=${RECAST_INSTALL}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install --config Release
)

# --------------------------------------------------------------------------
# libcarla_client
# --------------------------------------------------------------------------
set(CARLA_CLIENT_SRC ${PROJECT_SOURCE_DIR}/extern/carla)
set(CARLA_INSTALL_DIR ${CARLA_EXTERN_DIR}/carla/install)
# Patches applied to the CARLA client source:
#   carla-client-mutex.patch - add missing <mutex> includes (libstdc++/system Boost)
#   carla-client-cmake.patch - skip installing Boost and libpng headers/libs
#                             (both are provided system-wide)
set(CARLA_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/carla/carla-client-mutex.patch)
set(CARLA_PATCH_CMAKE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/carla/carla-client-cmake.patch)

ExternalProject_Add(carla_client
    DEPENDS carla_rpclib carla_recast
    # Configure directly against LibCarla/cmake. CARLA's top-level CMakeLists.txt
    # includes a generated Build/CMakeLists.txt.in (produced by `make setup`,
    # which requires UE4) that is absent from the 0.9.16 source checkout.
    SOURCE_DIR ${CARLA_CLIENT_SRC}/LibCarla/cmake
    BINARY_DIR ${CMAKE_BINARY_DIR}/extern/carla
    PREFIX ${CMAKE_BINARY_DIR}/extern/carla/prefix
    INSTALL_DIR ${CARLA_INSTALL_DIR}
    # Apply both patches in the CARLA source root (git -C), since the patch
    # paths are relative to the repo root, not to LibCarla/cmake.
    # Reset first so the patch step is idempotent (ExternalProject re-runs it
    # on every build; a dirty tree would otherwise fail the apply).
    PATCH_COMMAND git -C ${CARLA_CLIENT_SRC} checkout -- .
                 && git -C ${CARLA_CLIENT_SRC} apply ${CARLA_PATCH}
                 && git -C ${CARLA_CLIENT_SRC} apply ${CARLA_PATCH_CMAKE}
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Client
        # Artery loads this static library into OMNeT++ shared libraries, so
        # libcarla itself has to be position independent
        -DCMAKE_CXX_FLAGS=-fPIC
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DLIBCARLA_BUILD_RELEASE=ON
        -DLIBCARLA_BUILD_DEBUG=OFF
        -DLIBCARLA_BUILD_TEST=OFF
        -DCARLA_VERSION=0.9.16
        -DBOOST_INCLUDE_PATH=${Boost_INCLUDE_DIR}
        -DRPCLIB_INCLUDE_PATH=${RPCLIB_INSTALL}/include
        -DRPCLIB_LIB_PATH=${RPCLIB_INSTALL}/lib
        -DRECAST_INCLUDE_PATH=${RECAST_INSTALL}/include
        -DRECAST_LIB_PATH=${RECAST_INSTALL}/lib
        -DLIBPNG_INCLUDE_PATH=${PNG_INCLUDE_DIRS}
        -DCMAKE_INSTALL_PREFIX=${CARLA_INSTALL_DIR}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install --config Release
)

# --------------------------------------------------------------------------
# Imported targets
# --------------------------------------------------------------------------
add_library(Carla::Client STATIC IMPORTED GLOBAL)
set_target_properties(Carla::Client PROPERTIES
    IMPORTED_LOCATION ${CARLA_INSTALL_DIR}/lib/libcarla_client.a
    INTERFACE_INCLUDE_DIRECTORIES ${CARLA_INSTALL_DIR}/include
)
add_dependencies(Carla::Client carla_client)

add_library(Carla::Rpc STATIC IMPORTED GLOBAL)
set_target_properties(Carla::Rpc PROPERTIES
    IMPORTED_LOCATION ${RPCLIB_INSTALL}/lib/librpc.a
    INTERFACE_INCLUDE_DIRECTORIES ${RPCLIB_INSTALL}/include
)

add_library(Carla::Recast STATIC IMPORTED GLOBAL)
set_target_properties(Carla::Recast PROPERTIES
    IMPORTED_LOCATION ${RECAST_INSTALL}/lib/libRecast.a
    INTERFACE_INCLUDE_DIRECTORIES ${RECAST_INSTALL}/include
)

# Detour/DetourCrowd/DetourTileCache must be resolved *after* libcarla_client,
# which references them.  Listing them in INTERFACE_LINK_LIBRARIES of
# Carla::Recast is not enough: CMake moves static dependencies of INTERFACE
# libraries to the end of the link line, where they no longer satisfy the
# references of libcarla_client.a.  Consumers therefore have to list the
# archives explicitly (see src/carla/CMakeLists.txt).
set(CARLA_RECAST_LIBRARIES
    ${RECAST_INSTALL}/lib/libRecast.a
    ${RECAST_INSTALL}/lib/libDetourCrowd.a
    ${RECAST_INSTALL}/lib/libDetourTileCache.a
    ${RECAST_INSTALL}/lib/libDetour.a
    CACHE INTERNAL "CARLA Recast & Detour archives, in link order")
