cmake_minimum_required(VERSION 2.6.0)

find_package(TooN REQUIRED)
include_directories(${TOON_INCLUDE_PATHS})

find_package(Pangolin 0.3 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(SuiteSparse  REQUIRED)
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${CHOLMOD_INCLUDE_DIR})

find_package(Eigen3   REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(CUDA)
find_package(OpenNI2)
find_package(GLUT)    

ADD_DEFINITIONS(-fpermissive)

SET(appname efusion)

# --- common deps


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warning.



include_directories(./Core/src)
include_directories(./GUI/src)
include_directories(./GUI/src/Tools)




file(GLOB srcs Core/src/*.cpp)
file(GLOB utils_srcs Core/src/Utils/*.cpp)
file(GLOB shader_srcs Core/src/Shaders/*.cpp)
file(GLOB containers Core/src/Cuda/containers/*.cpp)
file(GLOB cuda Core/src/Cuda/*.cu)
file(GLOB gui_srcs       GUI/src/*.cpp)
file(GLOB gui_tools_srcs GUI/src/Tools/*.cpp)

IF(CUDA_FOUND)   

    STRING(REPLACE "_static.a" ".so" CUDA_LIBRARIES "${CUDA_LIBRARIES}")

    
    SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

    include_directories(${CUDA_INCLUDE_DIRS})

    SET(CUDA_ARCH_BIN "20 30 35 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
    SET(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              
    
    
    
    ## equiv --use_fast_math   without --fmad=true
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "--ftz=true;--prec-div=false;--prec-sqrt=false") 
    
	set(efusion_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Core/src/Shaders/" CACHE PATH "Where the shaders live")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall -std=c++11 -Wno-error=unused-parameter -Defusion_SHADER_DIR=${efusion_SHADER_DIR}")
    
    cuda_add_library(${appname}-cuda      SHARED  efusion_library.cpp
                ${srcs}
                ${utils_srcs}
                ${shader_srcs}
                ${cuda} 
                ${containers} OPTIONS -O3 -std=c++11 -Xcompiler -fPIC)
   
    
    target_link_libraries(${appname}-cuda 
                          ${Pangolin_LIBRARIES}
                            "-Wl,--no-as-needed;${CUDA_LIBRARIES};-Wl,--as-needed"
                           ${CHOLMOD_LIBRARIES}
    )

    
    generate_slam_library(${appname}-cuda)
    
        
ENDIF(CUDA_FOUND)

