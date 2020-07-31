cmake_minimum_required(VERSION 3.16)
project(astra_dataset)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "/home/turtlebot/lib/Astra_sdk/Astra_sdk/samples/cmake/")
set(ASTRA_SAMPLES_COMMON_DIR "/home/turtlebot/lib/Astra_sdk/Astra_sdk/samples/common/")
set(OPENNI_INCLUDE "/home/turtlebot/lib/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.7.10/Include")
set(OPENNI_LIB "/home/turtlebot/lib/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.7.10/Lib")
set(JSON_INCLUDE "/home/turtlebot/lib/json/include/nlohmann")
#MESSAGE( STATUS "AFTER:" )
#foreach(path ${CMAKE_MODULE_PATH})
#    message("     Path = " ${path})
#endforeach(path)

find_package(Astra REQUIRED)
find_package(OpenCV REQUIRED)

#MESSAGE( STATUS "CMAKE_MODULE_PATH:         " ${CMAKE_MODULE_PATH} )

add_executable(astra_dataset main.cpp)

include_directories(
        ${ASTRA_INCLUDE_DIR}
        ${ASTRA_SAMPLES_COMMON_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${OPENNI_INCLUDE}
        ${JSON_INCLUDE}
)

target_link_libraries(astra_dataset
        ${ASTRA_LIB_DIR}
        ${ASTRA_LIBRARIES}
        ${OpenCV_LIBS}
        ${OPENNI_LIB}
        )