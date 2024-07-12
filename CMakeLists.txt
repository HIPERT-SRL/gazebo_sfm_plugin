cmake_minimum_required(VERSION 3.5)
project(gazebo_sfm_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
if("$ENV{GZ_VERSION}" STREQUAL "garden")
    find_package(gz-sim7 REQUIRED)
    find_package(gz-rendering7 REQUIRED)
    find_package(gz-sensors7 REQUIRED)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
    set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
    set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
    find_package(gz-sim8 REQUIRED)
    find_package(gz-rendering8 REQUIRED)
    find_package(gz-sensors8 REQUIRED)

    set(GZ_TARGET_PREFIX gz)
    set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
    set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})
    set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

    message(STATUS "Compiling against Gazebo Harmonic")
else()
    message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()


add_library(PedestrianSFMPlugin SHARED
    src/PedestrianSFMPlugin.cpp
)
target_include_directories(PedestrianSFMPlugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(PedestrianSFMPlugin PUBLIC
    ${GZ_TARGET_PREFIX}-sim${GZ_SIM_VER}::core
    ${GZ_TARGET_PREFIX}-plugin${GZ_PLUGIN_VER}
    ${GZ_TARGET_PREFIX}-rendering${GZ_RENDERING_VER}
    ${GZ_TARGET_PREFIX}-sensors${GZ_SENSORS_VER}
    ${GZ_TARGET_PREFIX}-utils${GZ_UTILS_VER}
)
install(
  TARGETS PedestrianSFMPlugin
  DESTINATION lib
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
