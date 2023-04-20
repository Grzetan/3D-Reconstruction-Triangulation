cmake_minimum_required(VERSION 2.8)
project( 3D-Drone-Reconstruction )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(EXE_NAME main)
set(PATH "./src")
set(SOURCE_FILES
    ${PATH}/pugixml.hpp
    ${PATH}/pugixml.cpp
)

add_executable( ${EXE_NAME} ${PATH}/main.cpp ${SOURCE_FILES} )
target_link_libraries( ${EXE_NAME} ${OpenCV_LIBS} )
add_custom_command(TARGET ${EXE_NAME}
                   COMMAND ${EXE_NAME}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                   COMMENT "Running ${EXE_NAME}")