CMAKE_MINIMUM_REQUIRED( VERSION 3.0 )

project(KINECTTEST)

set(BIN_NAME knktest)
set(CMAKE_BUILD_TYPE Release)
set(SOURCE_FILES main.cpp)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    include_directories(${OpenCV_INCLUDE_DIRS} $ENV{OpenCV_DIR}/../../../include)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

add_executable(${BIN_NAME} ${SOURCE_FILES})
target_link_libraries(${BIN_NAME} ${OpenCV_LIBS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_CXX_FLAGS_RELEASE "/MT")
    set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
endif()
