cmake_minimum_required (VERSION 3.2)
project (planner_experiment)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)

set(EIGEN_LIB "common/eigen3")
set(OSQP_LIB "common/osqp")

include_directories({PYTHON_INCLUDE_DIRS} 
                    include/
                    common/
                    scenario/
                    ${EIGEN_LIB}
                    emplanner/include
                    ${OSQP_LIB}/include)
add_executable(${CMAKE_PROJECT_NAME}  
                main.cpp
                scenario/src/sce1.cpp
                scenario/src/sce2.cpp
                scenario/src/sce3.cpp
                common/cartesian_frenet_conversion.cc
                common/discrete_points_math.cc
                emplanner/src/em_planner.cpp
                emplanner/src/sl_dp.cpp
                emplanner/src/sl_qp.cpp
                emplanner/src/st_dp.cpp
                emplanner/src/st_qp.cpp)
target_link_directories(${CMAKE_PROJECT_NAME}   PRIVATE
  ${OSQP_LIB}/lib
)

target_link_libraries(${CMAKE_PROJECT_NAME} 
                        Python3::Python
                       Python3::NumPy
                       osqp)