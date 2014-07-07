include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PC_EIGEN_INCLUDE_DIRS}
)

link_directories(
)
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

orocos_component(KukaATICalibrationDemoRTNET kukaATICalibrationDemo-rtnetcomponent.cpp) # ...you may add multiple source files.

orocos_install_headers(kukaATICalibrationDemo-rtnetcomponent.hpp) # ...you may add multiple header files
