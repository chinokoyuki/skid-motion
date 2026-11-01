cmake_minimum_required(VERSION 3.15)

project(skid_motion VERSION 1.0.0 LANGUAGES C)

option(SKM_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(MSVC)
    add_compile_options(/W4 /std:c11)
else()
    add_compile_options(-std=c99 -Wall -Wextra -Wpedantic)
endif()
if(SKM_ENABLE_SANITIZERS AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
endif()

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(SKM_TOP_LEVEL ON)
else()
    set(SKM_TOP_LEVEL OFF)
endif()

option(SKM_BUILD_SIM "Build the host-side vehicle simulator" ${SKM_TOP_LEVEL})
option(SKM_BUILD_EXAMPLES "Build examples" ${SKM_TOP_LEVEL})

add_library(skid_motion
    src/skid_motion.c
    src/skid_pid.c
)
add_library(skid::motion ALIAS skid_motion)
set_target_properties(skid_motion PROPERTIES EXPORT_NAME motion)
target_include_directories(
    skid_motion
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(NOT MSVC)
    target_link_libraries(skid_motion PUBLIC m)
endif()

if(SKM_BUILD_SIM)
    add_library(skid_sim sim/skid_sim.c)
    add_library(skid::sim ALIAS skid_sim)
    set_target_properties(skid_sim PROPERTIES EXPORT_NAME sim)
    target_include_directories(
        skid_sim
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sim>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(skid_sim PUBLIC skid_motion)
endif()

if(SKM_BUILD_EXAMPLES AND SKM_BUILD_SIM)
    add_executable(skid_simulate examples/simulate.c)
    target_link_libraries(skid_simulate PRIVATE skid::motion skid::sim)
endif()

include(CTest)
if(BUILD_TESTING AND SKM_TOP_LEVEL)
    if(SKM_BUILD_SIM)
        add_subdirectory(tests)
    else()
        message(STATUS "Tests require SKM_BUILD_SIM=ON; test target is disabled")
    endif()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(SKM_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/skid_motion")
configure_package_config_file(
    cmake/skid_motion_config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/skid_motionConfig.cmake"
    INSTALL_DESTINATION "${SKM_CMAKE_INSTALL_DIR}"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/skid_motionConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS skid_motion EXPORT skid_motion_targets)
if(SKM_BUILD_SIM)
    install(TARGETS skid_sim EXPORT skid_motion_targets)
    install(FILES sim/skid_sim.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(
    EXPORT skid_motion_targets
    NAMESPACE skid::
    DESTINATION "${SKM_CMAKE_INSTALL_DIR}"
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/skid_motionConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/skid_motionConfigVersion.cmake"
    DESTINATION "${SKM_CMAKE_INSTALL_DIR}"
)
