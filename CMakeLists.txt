cmake_minimum_required(VERSION 3.8)
project(mesh_object)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/")
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/libomp")

# complie release
set(CMAKE_BUILD_TYPE Release)

# settings for complier version
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# pcl library
find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# cgal
find_package(CGAL REQUIRED COMPONENTS Core)

# OpenMP
find_package(OpenMP REQUIRED COMPONENTS CXX)

# MeshObject
set(MeshObject_SOURCE 
  src/MeshObject/Vertex.cpp
  src/MeshObject/Edge.cpp
  src/MeshObject/Face.cpp
  src/MeshObject/Surface.cpp
  src/MeshObject/Storage.cpp
  src/MeshObject/GenericPoint.cpp
  src/MeshObject/InteriorPoint.cpp
  src/MeshObject/TriangleBVH.cpp
  src/MeshObject/EdgeBVH.cpp
  src/MeshObject/RRSTree.cpp
  src/MeshObject/UnionFind.cpp
  src/MeshObject/Application.cpp
  src/MeshObject/InteractiveViewer.cpp
  src/MeshObject/Settings.cpp
  src/utilities/DataLoader.cpp
  src/utilities/covariance_math.cpp
  src/utilities/utilities.cpp
  src/MeshObject/Simulation.cpp
  src/utilities/omp_utilities.cpp
  src/utilities/simplified_mesh.cpp
)

# add executable
add_executable(main src/main.cpp ${MeshObject_SOURCE})
target_link_libraries (main ${PCL_LIBRARIES} CGAL::CGAL CGAL::CGAL_Core OpenMP::OpenMP_CXX)
target_include_directories(main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# target_compile_options(main PRIVATE -fsanitize=thread -g)
# target_link_options(main PRIVATE -fsanitize=thread)

# install(TARGETS main DESTINATION lib/${PROJECT_NAME})
install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}
)