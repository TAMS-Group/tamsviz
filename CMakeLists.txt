# TAMSVIZ
# (c) 2020 Philipp Ruppel

project(tamsviz)

set(CMAKE_CXX_FLAGS "-std=c++11 -g ")

find_package(OpenCV REQUIRED)

find_package(ASSIMP REQUIRED)

#set(ROS_BUILD_TYPE RelWithDebInfo)

#find_package(Qt5PlatformSupport REQUIRED)

find_package(Qt5 REQUIRED Core Widgets OpenGL)
set(QT_LIBRARIES Qt5::Widgets Qt5::OpenGL)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    roscpp
    std_msgs
    resource_retriever
    cv_bridge
    rosbag
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_robot_interaction
        roscpp
        std_msgs
        resource_retriever
        cv_bridge
        rosbag
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(tamsviz
    src/annotations/imageannotations.cpp
    src/core/bagplayer.cpp
    src/core/document.cpp
    src/core/event.cpp
    src/core/history.cpp
    src/core/loader.cpp
    src/core/log.cpp
    src/core/serialization.cpp
    src/core/snapshot.cpp
    src/core/topic.cpp
    src/core/transformer.cpp
    src/core/type.cpp
    src/core/workspace.cpp
    src/displays/axes.cpp
    src/displays/frame.cpp
    src/displays/grid.cpp
    src/displays/light.cpp
    src/displays/marker.cpp
    src/displays/material.cpp
    src/displays/materialrenderer.cpp
    src/displays/mesh.cpp
    src/displays/meshrenderer.cpp
    src/displays/robot.cpp
    src/displays/shapes.cpp
    src/displays/transform.cpp
    src/gui/displaytree.cpp
    src/gui/glqtwrapper.cpp
    src/gui/guicommon.cpp
    src/gui/imagewindow.cpp
    src/gui/mainwindow.cpp
    src/gui/plotwindow.cpp
    src/gui/propertygrid.cpp
    src/gui/renderthread.cpp
    src/gui/renderwindow.cpp
    src/gui/scenewindow.cpp
    src/gui/splitwindow.cpp
    src/gui/timeline.cpp
    src/render/context.cpp
    src/render/mesh.cpp
    src/render/renderer.cpp
    src/render/renderlist.cpp
    src/render/resource.cpp
    src/render/shader.cpp
    src/render/texture.cpp
    src/render/transformations.cpp
    src/render/uniformbuffer.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${QT_LIBRARIES} ${ASSIMP_LIBRARIES} -lGL -lEGL -lGLEW yaml-cpp)
