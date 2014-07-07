#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

set(CMAKE_BUILD_TYPE Release)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(kuka_ATI_calibration_demo)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()


find_package(PkgConfig REQUIRED)

PKG_CHECK_MODULES(PC_EIGEN eigen_lgsm REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( include )
set(CMAKE_CXX_FLAGS "-fpermissive")
add_subdirectory( src )

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
