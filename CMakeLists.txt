cmake_minimum_required(VERSION 3.16)
project(cpp_seg_msckf_vio CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_RERUN "Enable optional Rerun logging support" OFF)
set(RERUN_SDK_URL
    "https://github.com/rerun-io/rerun/releases/download/0.30.2/rerun_cpp_sdk.zip"
    CACHE STRING "URL for the Rerun C++ SDK bundle")

# ---- dependencies ----
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time)

if(ENABLE_RERUN)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL ${RERUN_SDK_URL})
    FetchContent_MakeAvailable(rerun_sdk)
endif()

# ---- include dirs ----
include_directories(include)

# ---- library: vio_core ----
add_library(vio_core
    src/io/euroc_reader.cpp
    src/io/kitti_raw_reader.cpp
    src/lidar/lidar_segmenter.cpp
    src/depth/stereo_depth_segmenter.cpp
    src/frontend/stereo_tracker.cpp
    src/ekf/imu_propagator.cpp
    src/ekf/lc_ekf.cpp
    # ---- msckf port (P1: types + utils + cam) ----
    src/msckf/types/Landmark.cpp
    src/msckf/utils/print.cpp
    # ---- msckf port (P2: state + propagator + statehelper) ----
    src/msckf/state/State.cpp
    src/msckf/state/Propagator.cpp
    src/msckf/state/StateHelper.cpp
    # ---- msckf port (P3: feat + update) ----
    src/msckf/feat/Feature.cpp
    src/msckf/feat/FeatureInitializer.cpp
    src/msckf/update/UpdaterHelper.cpp
    src/msckf/update/UpdaterMSCKF.cpp
    # ---- msckf pipeline adapter (P4) ----
    src/msckf_pipeline/msckf_pipeline.cpp
)
target_link_libraries(vio_core
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        yaml-cpp
        Boost::filesystem
        Boost::date_time
)
target_compile_definitions(vio_core PUBLIC ROS_AVAILABLE=0)

# ---- executables ----
add_executable(run_vio apps/run_vio.cpp)
target_link_libraries(run_vio PRIVATE vio_core)
if(ENABLE_RERUN)
    target_link_libraries(run_vio PRIVATE rerun_sdk)
    target_compile_definitions(run_vio PRIVATE LC_VIO_WITH_RERUN=1)
endif()

# Backward-compatible executable name; both dataset types are handled by run_vio.cpp.
add_executable(run_euroc apps/run_vio.cpp)
target_link_libraries(run_euroc PRIVATE vio_core)
if(ENABLE_RERUN)
    target_link_libraries(run_euroc PRIVATE rerun_sdk)
    target_compile_definitions(run_euroc PRIVATE LC_VIO_WITH_RERUN=1)
endif()

# Debug tool — P6 cycle 3 H1 (KITTI init-window stationarity probe).
add_executable(probe_kitti_imu tools/probe_kitti_imu_variance.cpp)
target_link_libraries(probe_kitti_imu PRIVATE vio_core)
