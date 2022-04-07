cmake_minimum_required (VERSION 3.11)
PROJECT(DroneFly)

FILE(GLOB SOURCE_FILE *.cpp *.h)

find_package(Pangolin CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(cpr CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_options(${PROJECT_NAME} PRIVATE "/FORCE:MULTIPLE")

target_include_directories(${PROJECT_NAME} PRIVATE ../../include)
target_include_directories(${PROJECT_NAME} PRIVATE ../voro)

target_link_libraries(${PROJECT_NAME} PRIVATE Voro)
target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE pangolin)
target_link_libraries(${PROJECT_NAME} PRIVATE jsoncpp_lib)
target_link_libraries(${PROJECT_NAME} PRIVATE cpr::cpr)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)
link_general(${PROJECT_NAME})
link_airsim(${PROJECT_NAME})