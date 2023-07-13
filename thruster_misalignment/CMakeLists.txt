cmake_minimum_required(VERSION 3.10)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/glog/cmake/) 
project(drake_cmake_test)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find_package(Threads MODULE REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
include_directories(include/)

add_executable(test_hcw test/run_hcw_dynamics.cc src/lib_dynamics.cc)
target_link_libraries(test_hcw drake::drake)

add_executable(test_kf test/run_kf.cc src/lib_kalman_filter.cc)
target_link_libraries(test_kf drake::drake)

add_executable(test_kf_hcw test/run_kf_dynamics.cc src/lib_dynamics.cc src/lib_kalman_filter.cc)
target_link_libraries(test_kf_hcw drake::drake)

add_executable(test_kf_bp test/run_kf_bp.cc src/lib_dynamics.cc src/lib_kalman_filter.cc)
target_link_libraries(test_kf_bp drake::drake)

add_executable(run_example_qp test/run_example_qp.cc)
target_link_libraries(run_example_qp drake::drake)