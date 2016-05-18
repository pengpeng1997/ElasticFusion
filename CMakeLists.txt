cmake_minimum_required(VERSION 2.6.0)

find_package(Pangolin 0.3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(SuiteSparse REQUIRED)

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

SET(appname efusion)

include_directories(./Core/src)
include_directories(./GUI/src)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${SUITESPARSE_INCLUDE_DIRS})


file(GLOB srcs Core/src/*.cpp)
file(GLOB utils_srcs Core/src/Utils/*.cpp)
file(GLOB shader_srcs Core/src/Shaders/*.cpp)
file(GLOB cuda Core/src/Cuda/*.cu)
file(GLOB containers Core/src/Cuda/containers/*.cpp)


  

set(CUDA_ARCH_BIN "20 30 35 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")           
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") 

CUDA_COMPILE(cuda_objs ${cuda})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall -std=c++11 -Wno-error=unused-parameter ")

add_library(${appname}-original  
            ${srcs}
            ${utils_srcs}
            ${shader_srcs}
            ${cuda} 
            ${cuda_objs} 
            ${containers}
)


target_link_libraries(${appname}-original
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${SUITESPARSE_LIBRARIES}
		      ${CSPARSE_LIBRARY}
		      cholmod
)

    add_benchmark(${appname}      original "" "")
    add_main     (${appname}      original "" "" "")
    
  install(TARGETS ${appname}-original  DESTINATION  lib/)

 # ----------------- GUI ----------------- 
	
find_package(ZLIB REQUIRED)
find_package(OpenNI2 REQUIRED)

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})

file(GLOB srcs       GUI/src/*.cpp)
file(GLOB tools_srcs GUI/src/Tools/*.cpp)


add_definitions(-Dlinux=1)

add_executable(ElasticFusion 
               ${srcs}
               ${tools_srcs}
)
install(TARGETS ElasticFusion  DESTINATION  bin/)

target_link_libraries(ElasticFusion
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${appname}-original
                      ${OPENNI2_LIBRARY}
)




