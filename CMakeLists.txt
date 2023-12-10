cmake_minimum_required(VERSION 3.5)

project(Door_Opening VERSION 1.0.0 LANGUAGES CXX)

# Activate C++ 11
set (CMAKE_CXX_STANDARD 11)


include_directories(thirdParty/cxxopts/)

add_compile_options(-Wall)
add_compile_options(-Wno-reorder)

add_definitions(-D_OS_UNIX)


link_libraries(${PROJECT_SOURCE_DIR}/lib/release/libKortexApiCpp.a)

# Add Include Directories
include_directories(include)
include_directories(include/client)
include_directories(include/common)
include_directories(include/messages)
include_directories(include/client_stubs)
include_directories(include/google)


# link other libs

link_libraries(pthread)


# Create executable for each example
# Look for examples under folders
file(GLOB EXE_LIST RELATIVE ${PROJECT_SOURCE_DIR} "src/*.cpp")
list(REMOVE_ITEM EXE_LIST "src/utilities.cpp")

foreach ( SRC_FILE ${EXE_LIST} )

  string(REPLACE ".cpp" "" TARGET_EXE_NAME ${SRC_FILE})
  string(REPLACE "/" "_" TARGET_EXE_NAME ${TARGET_EXE_NAME})
  
  MESSAGE("creating TARGET_EXE_NAME: '${TARGET_EXE_NAME}'")

  add_executable(${TARGET_EXE_NAME} ${SRC_FILE} src/utilities.cpp)

  
endforeach()
