CMAKE_MINIMUM_REQUIRED( VERSION 3.0 )

set( PROJECT_NAME kinect-test )
set( BIN_NAME kinect )
set( CMAKE_BUILD_TYPE Release )
set( SOURCE_FILES main.cpp )

project( ${PROJECT_NAME} )
find_package( OpenCV REQUIRED )
add_executable( ${BIN_NAME} ${SOURCE_FILES} )

include_directories( ${OpenCV_DIR}/../../../include )
target_link_libraries( ${BIN_NAME} ${OpenCV_LIBS} )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin )

