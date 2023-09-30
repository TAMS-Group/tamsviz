cmake_minimum_required(VERSION 3.1)
# TAMSVIZ
# (c) 2020-2023 Philipp Ruppel

project(tamsviz)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -g")

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

find_package(ASSIMP REQUIRED)

set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(Qt5 REQUIRED Core Widgets OpenGL)
set(QT_LIBRARIES Qt5::Widgets Qt5::OpenGL)

find_package(catkin REQUIRED COMPONENTS
    camera_info_manager
    cv_bridge
    eigen_conversions
    geometry_msgs
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    resource_retriever
    rosbag
    roscpp
    std_msgs
    laser_geometry
    tf
    image_transport
    pybind11_catkin
)

catkin_package(
    CATKIN_DEPENDS
        camera_info_manager
        cv_bridge
        geometry_msgs
        interactive_markers
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_robot_interaction
        resource_retriever
        rosbag
        roscpp
        std_msgs
        eigen_conversions
        laser_geometry
        pybind11_catkin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS program_options)

set(LIBRARY_NAME "tamsviz_core")
add_library(${LIBRARY_NAME}
    src/annotations/image.cpp
    src/annotations/scene.cpp
    src/components/environment.cpp
    src/components/rendering.cpp
    src/core/bagplayer.cpp
    src/core/component.cpp
    src/core/document.cpp
    src/core/event.cpp
    src/core/handle.cpp
    src/core/history.cpp
    src/core/interaction.cpp
    src/core/loader.cpp
    src/core/log.cpp
    src/core/message.cpp
    src/core/mparser.cpp
    src/core/mquery.cpp
    src/core/object.cpp
    src/core/profiler.cpp
    src/core/property.cpp
    src/core/serialization.cpp
    src/core/snapshot.cpp
    src/core/struct.cpp
    src/core/timeseries.cpp
    src/core/topic.cpp
    src/core/tracks.cpp
    src/core/transformer.cpp
    src/core/type.cpp
    src/core/variant.cpp
    src/core/watcher.cpp
    src/core/workspace.cpp
    src/displays/axes.cpp
    src/displays/camera.cpp
    src/displays/drvcam.cpp
    src/displays/frame.cpp
    src/displays/geometry.cpp
    src/displays/grid.cpp
    src/displays/interactive.cpp
    src/displays/laserscan.cpp
    src/displays/light.cpp
    src/displays/marker.cpp
    src/displays/mesh.cpp
    src/displays/plot.cpp
    src/displays/pointcloud.cpp
    src/displays/robot.cpp
    src/displays/shapes.cpp
    src/displays/text.cpp
    src/displays/transform.cpp
    src/gui/displaytree.cpp
    src/gui/guicommon.cpp
    src/gui/imagewindow.cpp
    src/gui/mainwindow.cpp
    src/gui/plotwindow.cpp
    src/gui/propertygrid.cpp
    src/gui/renderthread.cpp
    src/gui/renderwindow.cpp
    src/gui/scenewindow.cpp
    src/gui/searchwidget.cpp
    src/gui/splitwindow.cpp
    src/gui/timeline.cpp
    src/render/framebuffer.cpp
    src/render/imageloader.cpp
    src/render/mesh.cpp
    src/render/opengl.cpp
    src/render/renderbuffer.cpp
    src/render/renderer.cpp
    src/render/renderlist.cpp
    src/render/rendertarget.cpp
    src/render/resource.cpp
    src/render/shader.cpp
    src/render/texture.cpp
    src/render/transformations.cpp
    src/render/uniformbuffer.cpp
    src/scene/material.cpp
    src/scene/mesh.cpp
    src/scene/node.cpp
)
add_dependencies(${LIBRARY_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${QT_LIBRARIES} ${ASSIMP_LIBRARIES} -lGL -lEGL -lGLEW yaml-cpp)


add_executable(${PROJECT_NAME}
    src/gui/main.cpp
)
add_dependencies(${PROJECT_NAME} ${LIBRARY_NAME})
target_link_libraries(${PROJECT_NAME} ${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${QT_LIBRARIES} ${ASSIMP_LIBRARIES} -lGL -lEGL -lGLEW yaml-cpp)


# pybind_add_module(pytamsviz MODULE
#     src/python/python.cpp
# )
# add_dependencies(pytamsviz ${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${QT_LIBRARIES} ${ASSIMP_LIBRARIES} -lGL -lEGL -lGLEW yaml-cpp)
# target_link_libraries(pytamsviz PRIVATE ${LIBRARY_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} )
# set_target_properties(pytamsviz PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
