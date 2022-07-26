cmake_minimum_required (VERSION 3.11)
PROJECT(IBS)

FILE(GLOB SOURCE_FILE *.cpp *.h)
add_library(${PROJECT_NAME} ${SOURCE_FILE})
target_include_directories(${PROJECT_NAME} PRIVATE ../voro)
target_link_libraries(${PROJECT_NAME} PRIVATE Voro)
link_general(${PROJECT_NAME})