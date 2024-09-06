cmake_minimum_required(VERSION 3.16)
project(stand_go2)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(stand_go2 stand_go2.cpp)
add_executable(low_level low_level.cpp) 
add_executable(low_swing low_swing.cpp unitreeLeg.cpp)
add_executable(low_swingif low_swingif.cpp unitreeLeg.cpp)
add_executable(stand_go2fk stand_go2fk.cpp unitreeLeg.cpp)
add_executable(stand_go2if stand_go2if.cpp unitreeLeg.cpp)
add_executable(stand_go2free stand_go2free.cpp unitreeLeg.cpp)

SET(CMAKE_BUILD_TYPE Release)