cmake_minimum_required(VERSION 3.10)
project(eskf_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
)
FetchContent_MakeAvailable(googletest)

find_package(Eigen3 REQUIRED)

include("../cmake/torch.cmake")
list(APPEND CMAKE_PREFIX_PATH ${TORCH_CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH})
find_package(Torch REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${TORCH_INCLUDE_DIRS} ../include)

add_library(eskf STATIC ../src/liw/eskfEstimator.cpp)
target_include_directories(eskf PRIVATE ../include)

add_executable(test_eskf_estimator test_eskf_estimator.cpp)
target_link_libraries(test_eskf_estimator eskf gtest_main)

enable_testing()
add_test(NAME eskf_estimator_test COMMAND test_eskf_estimator)

add_executable(test_camera_utils test_camera_utils.cpp)
target_link_libraries(test_camera_utils gtest_main Eigen3::Eigen ${TORCH_LIBRARIES})
add_test(NAME camera_utils_test COMMAND test_camera_utils)
