cmake_minimum_required(VERSION 3.3)
project(3DU)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++ ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(SOURCE_FILES  Main.h Main.cpp Shader.h Camera.h Model.h Model.cpp Triangulate.h Triangulate.cpp common.h fileOperations.h fileOperations.cpp)
set(SOURCE_FILES  ImageStitching.cpp ImageStitching.h)
add_executable(3DU ${SOURCE_FILES})

add_custom_command(TARGET 3DU POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data ${CMAKE_BINARY_DIR}/data)

add_custom_command(TARGET 3DU POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders ${CMAKE_BINARY_DIR}/shaders)

# load libraries
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH})

find_package(GLFW REQUIRED)
if (GLFW_FOUND)
    include_directories(${GLFW_INCLUDE_DIR})
    target_link_libraries(3DU ${GLFW_LIBRARY})
endif ()

# openGL extra library
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    target_link_libraries(3DU ${GLEW_LIBRARIES})
endif ()

find_package(SOIL REQUIRED)
if (SOIL_FOUND)
    include_directories(${SOIL_INCLUDE_PATH})
    target_link_libraries(3DU ${SOIL_LIBRARIES})
endif()

find_package(PCL REQUIRED)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_PATH})
    target_link_libraries(3DU ${PCL_LIBRARIES})
endif()

find_package(EIGEN REQUIRED)
if (EIGEN_FOUND)
    include_directories(${EIGEN_INCLUDE_PATH})
    target_link_libraries(3DU ${EIGEN_LIBRARIES})
endif()

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_PATH})
    target_link_libraries(3DU ${OpenCV_LIBRARIES})
endif()

find_package(VTK REQUIRED)
if (VTK_FOUND)
    include_directories(${VTK_INCLUDE_PATH})
    target_link_libraries(3DU ${VTK_LIBRARIES})
endif()

if (APPLE)
    target_link_libraries(3DU "-framework OpenGL")
endif ()