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


# Put all the runtime stuff in the same directory.  By default, CMake puts each targets'
# output into their own directory.  We want all the targets to be put in the same
# directory, and we can do this by setting these variables.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

find_package(OptiX REQUIRED)

include_directories(
  "${OptiX_INCLUDE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/cuda"
  )

##################################################################
# SUtil compilation

set(SAMPLES_PTX_DIR "${CMAKE_BINARY_DIR}/lib/ptx")
set(SAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

set(CUDA_GENERATED_OUTPUT_DIR ${SAMPLES_PTX_DIR})

if( WIN32 )
  string(REPLACE "/" "\\\\" SAMPLES_PTX_DIR ${SAMPLES_PTX_DIR})
else( WIN32 )
  if( USING_GNU_C AND NOT APPLE )
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DM_PI=3.14159265358979323846" )
  endif()
endif( WIN32 )

set(SAMPLES_CUDA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cuda")

set(SAMPLES_SUPPORT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SDK/support")

# NVRTC include paths relative to the sample path
set(SAMPLES_RELATIVE_INCLUDE_DIRS "\\
  \"cuda\", \\
  \"sutil\", \\
  \"lib/DemandLoading/include\", \\
  \"lib/ImageSource/include\", \\
  \".\", ")

# NVRTC absolute include paths to the headers used to build the samples
set(SAMPLES_ABSOLUTE_INCLUDE_DIRS "\\
  \"${OptiX_INCLUDE}\", \\
  \"${CUDA_INCLUDE_DIRS}\", ")

# Build a null-terminated option list for NVRTC
set(CUDA_NVRTC_OPTIONS)
foreach(flag ${CUDA_NVRTC_FLAGS})
  set(CUDA_NVRTC_OPTIONS "${CUDA_NVRTC_OPTIONS} \\\n  \"${flag}\",")
endforeach()
set(CUDA_NVRTC_OPTIONS "${CUDA_NVRTC_OPTIONS}")
message(STATUS "${OptiX_INSTALL_DIR}")
configure_file("${OptiX_INSTALL_DIR}/SDK/sampleConfig.h.in" sampleConfig.h @ONLY)

# Path to sutil.h that all the samples need
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     "${CMAKE_BINARY_DIR}/include"
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${CUDA_INCLUDE_DIRS}
                     )

set(SAMPLES_CUDA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cuda)

# Helper macro to generate PTX from the CUDA files in sutil.
macro(OPTIX_sutil_compile_to_optix_input generated_files)
  if( SAMPLES_INPUT_GENERATE_OPTIXIR )
    message("sutil OPTIXIR")
    CUDA_WRAP_SRCS( sutil OPTIXIR generated_files2 ${ARGN} )
    list(APPEND ${generated_files} ${generated_files2})
  endif()
  if( SAMPLES_INPUT_GENERATE_PTX AND NOT CUDA_NVRTC_ENABLED)
    message("sutil PTX")
    CUDA_WRAP_SRCS( sutil PTX generated_files3 ${ARGN} )
    list(APPEND ${generated_files} ${generated_files3})
  endif()
  message("${generated_files} = ${${generated_files}}")
endmacro()

# These calls will group PTX and CUDA files into their own directories in the Visual
# Studio projects.
macro(OPTIX_add_source_groups)
  if( SAMPLES_INPUT_GENERATE_PTX )
    if( NOT CUDA_NVRTC_ENABLED )
      source_group("PTX Files"  REGULAR_EXPRESSION ".+\\.ptx$")
    endif()
  endif()
  if( SAMPLES_INPUT_GENERATE_OPTIXIR )
    source_group("OptixIR Files"  REGULAR_EXPRESSION ".+\\.optixir$")
  endif()
  source_group("CUDA Files" REGULAR_EXPRESSION ".+\\.cu$")
endmacro()

# NVRTC include paths relative to the sample path
set(SAMPLES_RELATIVE_INCLUDE_DIRS "\\
  \"cuda\", \\
  \"sutil\", \\
  \"lib/DemandLoading/include\", \\
  \"lib/ImageSource/include\", \\
  \".\", ")

# NVRTC absolute include paths to the headers used to build the samples
set(SAMPLES_ABSOLUTE_INCLUDE_DIRS "\\
  \"${OptiX_INCLUDE}\", \\
  \"${CUDA_INCLUDE_DIRS}\", ")

# Build a null-terminated option list for NVRTC
set(CUDA_NVRTC_OPTIONS)
foreach(flag ${CUDA_NVRTC_FLAGS})
  set(CUDA_NVRTC_OPTIONS "${CUDA_NVRTC_OPTIONS} \\\n  \"${flag}\",")
endforeach()
set(CUDA_NVRTC_OPTIONS "${CUDA_NVRTC_OPTIONS}")

configure_file("${OptiX_INSTALL_DIR}/SDK/sampleConfig.h.in" sampleConfig.h @ONLY)

# Path to sutil.h that all the samples need
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     "${CMAKE_BINARY_DIR}/include"
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${CUDA_INCLUDE_DIRS}
                     )

set(SAMPLES_CUDA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cuda)

# Helper macro to generate PTX from the CUDA files in sutil.
macro(OPTIX_sutil_compile_to_optix_input generated_files)
  if( SAMPLES_INPUT_GENERATE_OPTIXIR )
    message("sutil OPTIXIR")
    CUDA_WRAP_SRCS( sutil OPTIXIR generated_files2 ${ARGN} )
    list(APPEND ${generated_files} ${generated_files2})
  endif()
  if( SAMPLES_INPUT_GENERATE_PTX AND NOT CUDA_NVRTC_ENABLED)
    message("sutil PTX")
    CUDA_WRAP_SRCS( sutil PTX generated_files3 ${ARGN} )
    list(APPEND ${generated_files} ${generated_files3})
  endif()
  message("${generated_files} = ${${generated_files}}")
endmacro()

# These calls will group PTX and CUDA files into their own directories in the Visual
# Studio projects.
macro(OPTIX_add_source_groups)
  if( SAMPLES_INPUT_GENERATE_PTX )
    if( NOT CUDA_NVRTC_ENABLED )
      source_group("PTX Files"  REGULAR_EXPRESSION ".+\\.ptx$")
    endif()
  endif()
  if( SAMPLES_INPUT_GENERATE_OPTIXIR )
    source_group("OptixIR Files"  REGULAR_EXPRESSION ".+\\.optixir$")
  endif()
  source_group("CUDA Files" REGULAR_EXPRESSION ".+\\.cu$")
endmacro()

FILE(GLOB SOURCE_FILE *.h *.cu *.cpp *.cuh)

find_package(CUDAToolkit)
find_package(Threads)
find_package(nanoflann CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_options(${PROJECT_NAME} PUBLIC "/FORCE:MULTIPLE")
target_link_libraries(${PROJECT_NAME} PUBLIC CUDA::cudart)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES 75)

target_include_directories(${PROJECT_NAME} PUBLIC ../../include)

link_general(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE nanoflann::nanoflann)
target_include_directories(${PROJECT_NAME} PUBLIC ${Pangolin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE jsoncpp_lib jsoncpp_object)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)