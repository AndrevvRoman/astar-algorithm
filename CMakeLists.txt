cmake_minimum_required(VERSION 3.0.0)
project(astar-algorithm VERSION 0.1.0)


add_executable(astar-algorithm main.cpp)

find_package(SFML COMPONENTS system window graphics)

target_link_libraries(astar-algorithm sfml-graphics sfml-system sfml-window pthread)

file(COPY "fonts/" DESTINATION ${CMAKE_BINARY_DIR}/fonts )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

add_subdirectory(tests)
