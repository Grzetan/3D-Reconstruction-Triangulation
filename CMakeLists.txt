cmake_minimum_required(VERSION 2.8)
project( 3D-Drone-Reconstruction )
set(CMAKE_CXX_STANDARD 17)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(EXE_NAME main)
set(TEST_EXE_NAME test)
set(PATH "./src")
set(SOURCE_FILES
    ${PATH}/pugixml.hpp
    ${PATH}/pugixml.cpp
    ${PATH}/Triangulator.h
    ${PATH}/Triangulator.cpp
    ${PATH}/MatrixTriangulator.h
    ${PATH}/MatrixTriangulator.cpp
    ${PATH}/RayTriangulator.h
    ${PATH}/RayTriangulator.cpp
    ${PATH}/DroneClassifier.h
    ${PATH}/DroneClassifier.cpp
    ${PATH}/DetectionsContainer.h
    ${PATH}/DetectionsContainer.cpp
    ${PATH}/utils.h
    ${PATH}/utils.cpp
    ${PATH}/argparse.hpp
)
set(SOURCE_FILES_TEST
    ${PATH}/pugixml.hpp
    ${PATH}/pugixml.cpp
    ${PATH}/argparse.hpp
    ${PATH}/utils.h
    ${PATH}/utils.cpp
    ${PATH}/happly.h
    ${PATH}/Camera.h
)

add_executable( ${EXE_NAME} ${PATH}/main.cpp ${SOURCE_FILES} )
add_executable( ${TEST_EXE_NAME} ${PATH}/test.cpp ${SOURCE_FILES_TEST})
target_link_libraries( ${EXE_NAME} ${OpenCV_LIBS} )
target_link_libraries( ${TEST_EXE_NAME} ${OpenCV_LIBS} )

# add_custom_command(TARGET ${EXE_NAME}
#                    COMMAND ${EXE_NAME}
#                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#                    COMMENT "Running ${EXE_NAME}")