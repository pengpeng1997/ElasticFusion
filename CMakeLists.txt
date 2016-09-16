cmake_minimum_required(VERSION 2.6.0)

find_package(Pangolin 0.3 REQUIRED)
find_package(CUDA)
find_package(OpenNI2)
find_package(SuiteSparse REQUIRED)

SET(appname efusion)

include_directories(./Core/src)
include_directories(./GUI/src)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${SUITESPARSE_INCLUDE_DIRS})


file(GLOB srcs Core/src/*.cpp)
file(GLOB utils_srcs Core/src/Utils/*.cpp)
file(GLOB shader_srcs Core/src/Shaders/*.cpp)
file(GLOB cuda Core/src/Cuda/*.cu)
file(GLOB containers Core/src/Cuda/containers/*.cpp)

IF(CUDA_FOUND)   
    
    SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

    include_directories(${CUDA_INCLUDE_DIRS})

    SET(CUDA_ARCH_BIN "20 30 35 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
    SET(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              
    
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;-std=c++11")           
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") 
    
    CUDA_COMPILE(cuda_objs ${cuda})
    
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall -std=c++11 -Wno-error=unused-parameter ")
    
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
    		              ${CHOLMOD_LIBRARIES}
    )
    
    
    add_slambench_binary(${appname}  benchmark    original "" "-D_GLIBCXX_USE_CXX11_ABI=1" "")
    
IF(OpenNI2_FOUND) 

    add_slambench_binary(${appname}  main         original "" "-D_GLIBCXX_USE_CXX11_ABI=1" "")
    
    install(TARGETS ${appname}-original  DESTINATION  lib/)
    
    # ----------------- GUI ----------------- 
    	
    	
    find_package(ZLIB REQUIRED)
    
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
    
ENDIF(OpenNI2_FOUND)
ENDIF(CUDA_FOUND)

