cmake_minimum_required (VERSION 3.11)
PROJECT(AirSim)
set(AIRSIM_ROOT "C:/SSD/GitProject/AirSim")
INCLUDE_DIRECTORIES("${AIRSIM_ROOT}/AirLib/include")

function (link_airsim arg)
    target_link_libraries(${ARGV0} PRIVATE "${AIRSIM_ROOT}/AirLib/deps/rpclib/lib/x64/RelWithDebInfo/rpc.lib")
    target_link_libraries(${ARGV0} PRIVATE "${AIRSIM_ROOT}/AirLib/lib/x64/RelWithDebInfo/AirLib.lib")
endfunction ()

FILE(GLOB SOURCE_FILE *.cpp *.h)
add_library(${PROJECT_NAME} ${SOURCE_FILE})