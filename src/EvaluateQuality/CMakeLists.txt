cmake_minimum_required (VERSION 3.10)
PROJECT(EvaluateQuality)

FILE(GLOB SOURCE_FILE *.cpp *.h)

set(libpointmatcher_DIR "C:/SSD/GitProject/libpointmatcher/build/install/share/libpointmatcher/cmake")
set(libnabo_DIR "C:/SSD/GitProject/libnabo/build")

find_package(libpointmatcher CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})
target_include_directories(${PROJECT_NAME} PRIVATE ../../include)
target_include_directories(${PROJECT_NAME} PUBLIC ${libpointmatcher_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE jsoncpp_lib)
link_general(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE ${libpointmatcher_LIBRARIES})
