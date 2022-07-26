cmake_minimum_required (VERSION 3.18)
PROJECT(PathPlanning LANGUAGES CUDA CXX)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH
  "C:/ProgramData/NVIDIA Corporation/OptiX SDK 7.5.0/SDK/CMake"
  ${CMAKE_MODULE_PATH}
  )
# Locate the NVRT distribution.  Search the SDK first, then look in the system.
set(OptiX_INSTALL_DIR "C:/ProgramData/NVIDIA Corporation/OptiX SDK 7.5.0/")

# Include all CMake Macros.
include(Macros)
# Determine information about the compiler
include (CompilerInfo)

OPTION(CUDA_REMOVE_GLOBAL_MEMORY_SPACE_WARNING "Suppress the \"Advisory: Cannot tell what pointer points to, assuming global memory space\" warning nvcc makes." ON)

find_package(CUDA 11.0 REQUIRED)

if( CUDA_VERSION VERSION_GREATER_EQUAL 11.7 )
  option(SAMPLES_INPUT_ENABLE_OPTIXIR_SUPPORT "Enable support for generating OptiX-IR targetted input files" ON)
else()
  # You can try to force this on
  option(SAMPLES_INPUT_ENABLE_OPTIXIR_SUPPORT "Enable support for generating OptiX-IR targetted input files" OFF)
endif()

# Present the CUDA_64_BIT_DEVICE_CODE on the default set of options.
mark_as_advanced(CLEAR CUDA_64_BIT_DEVICE_CODE)

set(CUDA_MIN_SM_TARGET sm_50 CACHE STRING "Minimum CUDA SM architecture to use for compilation.")

find_package(OptiX REQUIRED)

include_directories(
  "${OptiX_INCLUDE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/cuda"
  )

#FILE(GLOB SOURCE_FILE *.h *.cu *.cpp *.cuh)

find_package(CUDAToolkit)
find_package(Threads)
find_package(nanoflann CONFIG REQUIRED)
#find_package(Pangolin CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)

add_executable(${PROJECT_NAME} CalRec.cu CalRec.cuh
dronescan.cpp dronescan.h
initialize.cpp initialize.h
optimize_trajectory.cpp
OptixUti.cu OptixUti.cuh
ppc.cpp ppc.h
SmithOptCuda.cu SmithOptCuda.cuh
SmithOptimized.cpp SmithOptimized.h 
)

target_link_options(${PROJECT_NAME} PUBLIC "/FORCE:MULTIPLE")
target_link_libraries(${PROJECT_NAME} PUBLIC CUDA::cudart)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES 75)

target_include_directories(${PROJECT_NAME} PUBLIC ../../include)

link_general(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE nanoflann::nanoflann)
#target_include_directories(${PROJECT_NAME} PUBLIC ${Pangolin_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE jsoncpp_lib jsoncpp_object)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)
#target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL_Basic_viewer)