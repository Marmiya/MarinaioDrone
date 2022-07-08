# CMakeList.txt : CMake project for Drone, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

project ("ImiRT")

find_package(nlohmann_json REQUIRED)
find_package(Pangolin CONFIG REQUIRED)

# Add source to this project's executable.

FILE(GLOB SOURCE_FILE *.cpp *.h)
add_executable (${PROJECT_NAME} ${SOURCE_FILE})

target_link_options(${PROJECT_NAME} PRIVATE "/FORCE:MULTIPLE")

target_include_directories(${PROJECT_NAME} PUBLIC ../../include)
target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} PRIVATE pangolin)

link_general(${PROJECT_NAME})
# TODO: Add tests and install targets if needed.