cmake_minimum_required(VERSION 3.0.0)
project(RTP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-lstdc++fs" )
set(CMAKE_CXX_STANDARD_REQUIRED ON )

include(CTest)
enable_testing()

add_executable(mfe mfe.cpp )

find_package( depthai REQUIRED static)
target_link_libraries(mfe depthai::opencv)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${RTP_VERSION})
include(CPack)
