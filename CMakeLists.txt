cmake_minimum_required(VERSION 3.15.3)
project(GLNSModified)

set(CMAKE_CXX_STANDARD 17)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/lib/x86_64-linux-gnu/pkgconfig")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")

## Boost.
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.52 COMPONENTS program_options system filesystem)

##Following only for profiling.
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")



include_directories("/usr/include/cairomm-1.0")

link_directories(
        ${GTKMM_LIBRARY_DIRS}
)

include_directories(
        ${GTKMM_INCLUDE_DIRS}
)


find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)

add_executable(GLNSModified main.cpp point.cpp point.h polygon.h tour.h planner.cpp planner.h heuristic.h heuristic.cpp clipper.hpp clipper.cpp MapDrawer.cpp MapDrawer.hpp utils.cpp utils.hpp geom.cpp geom.hpp poly_maps.hpp simple_intersection.h triangle.c triangle.h triangulation.hpp triangulation.cpp TPP.cpp TPP.h)

target_compile_definitions(GLNSModified
        PRIVATE -DTRILIBRARY -DANSI_DECLARATORS -DNO_TIMER)

target_link_libraries(GLNSModified
        ${GTKMM_LIBRARIES}
        )