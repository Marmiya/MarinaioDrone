cmake_minimum_required (VERSION 3.11)
PROJECT(DroneFly)

set(AIRSIM_ROOT "C:/SSD/GitProject/AirSim")
INCLUDE_DIRECTORIES("${AIRSIM_ROOT}/AirLib/include")

FILE(GLOB SOURCE_FILE *.cpp *.h)

find_package(Pangolin CONFIG REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(cpr CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(SCIP CONFIG REQUIRED)
find_package(CGAL COMPONENTS Qt5 REQUIRED)

add_definitions(-DCGAL_USE_BASIC_VIEWER)
include(CGAL_SCIP_support)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_options(${PROJECT_NAME} PRIVATE "/FORCE:MULTIPLE")

target_include_directories(${PROJECT_NAME} PRIVATE ../../include)
#target_include_directories(${PROJECT_NAME} PRIVATE ../voro)
target_include_directories(${PROJECT_NAME} PRIVATE ../AirSim)
target_include_directories(${PROJECT_NAME} PRIVATE ../IBS)

#target_link_libraries(${PROJECT_NAME} PRIVATE Voro)
target_link_libraries(${PROJECT_NAME} PRIVATE IBS)
target_link_libraries(${PROJECT_NAME} PRIVATE AirSim)
target_link_libraries(${PROJECT_NAME} PRIVATE AMarinaioDrone)
target_link_libraries(${PROJECT_NAME} PRIVATE pangolin)
target_link_libraries(${PROJECT_NAME} PRIVATE jsoncpp_lib)
target_link_libraries(${PROJECT_NAME} PRIVATE cpr::cpr)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)
target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL_Basic_viewer)
#target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::SCIP_support)
link_general(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE "${AIRSIM_ROOT}/AirLib/deps/rpclib/lib/x64/RelWithDebInfo/rpc.lib")
target_link_libraries(${PROJECT_NAME} PRIVATE "${AIRSIM_ROOT}/AirLib/lib/x64/RelWithDebInfo/AirLib.lib")