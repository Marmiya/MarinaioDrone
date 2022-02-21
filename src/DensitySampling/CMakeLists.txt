cmake_minimum_required (VERSION 3.10)
PROJECT(DensitySampling)

FILE(GLOB SOURCE_FILE *.cpp *.h)

find_package(Threads)
find_package(Pangolin CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC ../../include)
target_include_directories(${PROJECT_NAME} PUBLIC ${Pangolin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE glm::glm)
link_general(${PROJECT_NAME})