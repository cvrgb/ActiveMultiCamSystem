cmake_minimum_required(VERSION 2.8.8)

# Project name
project(ActiveBionicVision)


find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()
message(WARNING "OPENCV VERSION: " ${OpenCV_VERSION})


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer  -funroll-loops -Wno-sign-compare")
# Enable debug flags (use if you want to debug in gdb)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin/)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib/)

# Include our header files
include_directories( ${EIGEN3_INCLUDE_DIR}        ${OpenCV_INCLUDE_DIRS})

include_directories(./modules/apriltags/include)
aux_source_directory(./modules/apriltags/src APRILGRID_SRCS)

include_directories(./modules/calibrateCamera/include)
aux_source_directory(./modules/calibrateCamera/src CALIBRATECAMERA_SRCS)

include_directories(./modules/handeye/include)
aux_source_directory(./modules/handeye/src HANDEYE_SRCS)

include_directories(./modules/kinematics/include)
aux_source_directory(./modules/kinematics/src KINEMATICS_SRCS)

include_directories(./modules/common/include)
aux_source_directory(./modules/common/src COMMON_SRCS)

include_directories(./modules/thirdParty)
#aux_source_directory(./modules/common/src MANIF_SRCS)

if (on)
set(TEST_NAME_EXE calib_aprilgrid)
add_executable(${TEST_NAME_EXE} ./examples/${TEST_NAME_EXE}.cpp 
${APRILGRID_SRCS} ${CALIBRATECAMERA_SRCS} ${COMMON_SRCS})
target_link_libraries(${TEST_NAME_EXE}  ${OpenCV_LIBRARIES}  Ceres::ceres)
endif()

if (off)
set(TEST_NAME_EXE checkerboard)
add_executable(${TEST_NAME_EXE} ./examples/${TEST_NAME_EXE}.cpp 
${APRILGRID_SRCS} ${CALIBRATECAMERA_SRCS} ${COMMON_SRCS})
target_link_libraries(${TEST_NAME_EXE}  ${OpenCV_LIBRARIES}  Ceres::ceres)
endif()

if (on)
set(TEST_NAME_EXE app_handeye)
add_executable(${TEST_NAME_EXE} ./examples/${TEST_NAME_EXE}.cpp 
${APRILGRID_SRCS} ${CALIBRATECAMERA_SRCS} ${COMMON_SRCS} ${HANDEYE_SRCS})
target_link_libraries(${TEST_NAME_EXE}  ${OpenCV_LIBRARIES}  Ceres::ceres)
endif()

if (on)
set(TEST_NAME_EXE app_handeye_M1)
add_executable(${TEST_NAME_EXE} ./examples/${TEST_NAME_EXE}.cpp 
${APRILGRID_SRCS} ${CALIBRATECAMERA_SRCS} ${COMMON_SRCS} ${HANDEYE_SRCS} ${KINEMATICS_SRCS})
target_link_libraries(${TEST_NAME_EXE}  ${OpenCV_LIBRARIES}  Ceres::ceres)
endif()

if (on)
set(TEST_NAME_EXE app_stitch_wide)
add_executable(${TEST_NAME_EXE} ./examples/${TEST_NAME_EXE}.cpp 
${APRILGRID_SRCS} ${CALIBRATECAMERA_SRCS} ${COMMON_SRCS} ${HANDEYE_SRCS} ${KINEMATICS_SRCS})
target_link_libraries(${TEST_NAME_EXE}  ${OpenCV_LIBRARIES}   Ceres::ceres)
endif()
 
# ##################################################
# # Make the core library
# ##################################################
# add_library(abv_core_lib         
#         # ./src/feat/Feature.cpp
#         # ./src/feat/Checkerboard.cpp
#         # ./src/calib/Kinematic.cpp
#         ${ABV_CORE_SRCS}
#         ${CBDETECT_SRCS}      
#         #/home/be/Projects/active-binocular-vins_hawkeye/abv_core/src/calib/GridCalibrationTargetAprilgrid.cpp          
#         #src/feat/FeatureInitializer.cpp
# )

# target_include_directories(abv_core_lib PUBLIC 
# ./src 
# ./src/thirdparty
# /home/be/Projects/Repository/kalibr/aslam_offline_calibration/ethz_apriltag2/build/include
# # set(apriltag_LIBS /home/be/Projects/Repository/kalibr/aslam_offline_calibration/ethz_apriltag2/build/lib/libethz_apriltags.a)
# # include_directories(${apriltag_INCLUDE_DIR})
# )

 



