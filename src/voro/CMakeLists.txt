cmake_minimum_required (VERSION 3.11)
PROJECT(Voro)

FILE(GLOB SOURCE_FILE *.cc *.hh)
add_library(${PROJECT_NAME} ${SOURCE_FILE})