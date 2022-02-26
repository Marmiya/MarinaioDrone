cmake_minimum_required (VERSION 3.11)
PROJECT(PathPlanning LANGUAGES CUDA CXX)

set(CUDA_INCLUDE_DIRS "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.6/include" ${CUDA_INCLUDE_DIRS})

enable_language(CUDA)

FILE(GLOB SOURCE_FILE *.h *.cu *.cpp *.cuh)

find_package(Threads)
find_package(nanoflann CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_options(${PROJECT_NAME} PUBLIC "/FORCE:MULTIPLE")

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