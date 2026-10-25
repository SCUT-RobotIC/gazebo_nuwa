cmake_minimum_required(VERSION 3.16)
project(mid360_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Use the system protobuf (3.12) consistently. This machine also has a
# source-installed protobuf 3.6 under /usr/local, which CMake's FindProtobuf
# would otherwise pick first and mix with the headers gz-msgs was built with.
if(EXISTS "/usr/lib/x86_64-linux-gnu/libprotobuf.so")
  set(Protobuf_LIBRARY "/usr/lib/x86_64-linux-gnu/libprotobuf.so"
      CACHE FILEPATH "" FORCE)
  set(Protobuf_INCLUDE_DIR "/usr/include" CACHE PATH "" FORCE)
  set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc"
      CACHE FILEPATH "" FORCE)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(mid360_lidar_system SHARED src/Mid360Lidar.cc)

target_include_directories(mid360_lidar_system
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(TARGET protobuf::libprotobuf AND
   EXISTS "/usr/lib/x86_64-linux-gnu/libprotobuf.so.23")
  set_target_properties(protobuf::libprotobuf PROPERTIES
    IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libprotobuf.so.23"
    IMPORTED_LOCATION_RELEASE "/usr/lib/x86_64-linux-gnu/libprotobuf.so.23"
    IMPORTED_SONAME "libprotobuf.so.23")
endif()

# This machine has a source-installed protobuf under /usr/local (3.6), which
# shadows the system protobuf (3.12) that gz-msgs was generated against. GCC
# always searches /usr/local/include before /usr/include, and an explicit
# -I/usr/include is ignored because it is a standard system directory.
# A one-level shim makes <google/protobuf/...> resolve to the system headers.
if(EXISTS "/usr/include/google/protobuf/port_def.inc")
  set(MID360_PROTOBUF_SHIM "${CMAKE_CURRENT_BINARY_DIR}/protobuf_shim")
  file(MAKE_DIRECTORY "${MID360_PROTOBUF_SHIM}/google")
  file(REMOVE "${MID360_PROTOBUF_SHIM}/google/protobuf")
  file(CREATE_LINK "/usr/include/google/protobuf"
       "${MID360_PROTOBUF_SHIM}/google/protobuf" SYMBOLIC)
  target_include_directories(mid360_lidar_system BEFORE PRIVATE
    "${MID360_PROTOBUF_SHIM}")
endif()

target_compile_options(mid360_lidar_system PRIVATE -Wall -Wextra)

target_link_libraries(mid360_lidar_system
  PUBLIC
    gz-sim8::gz-sim8
    gz-msgs10::gz-msgs10
    gz-transport13::gz-transport13
    gz-plugin2::gz-plugin2
    gz-common5::gz-common5
    gz-math7::gz-math7
    ${CMAKE_DL_LIBS})

set_target_properties(mid360_lidar_system PROPERTIES
  OUTPUT_NAME mid360_lidar_system
  PREFIX "lib")

# colcon / ament install layout. The standalone build.sh only needs the .so in
# build/, because the plugin looks for ../patterns/mid360.csv relative to the
# shared library.
install(TARGETS mid360_lidar_system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib)
install(FILES patterns/mid360.csv
  DESTINATION share/mid360_sim/patterns)
