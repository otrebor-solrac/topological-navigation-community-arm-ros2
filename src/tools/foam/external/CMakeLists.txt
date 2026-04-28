add_subdirectory(spheretree)

find_package(Threads REQUIRED)

# Solo mantenemos las versiones actuales que si existen y se usan
add_executable(
  manifold
  manifold/src/GridIndex.h
  manifold/src/Intersection.h
  manifold/src/Intersection.cc
  manifold/src/IO.h
  manifold/src/IO.cc
  manifold/src/main.cc
  manifold/src/Manifold.h
  manifold/src/Manifold.cc
  manifold/src/MeshProjector.cc
  manifold/src/MeshProjector.h
  manifold/src/Octree.h
  manifold/src/Octree.cc
  manifold/src/Parser.h
  manifold/src/Parser.cc
  manifold/src/types.h
)

target_include_directories(manifold PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/manifold/src)
target_include_directories(manifold PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/manifold/3rd_party/eigen/)
target_include_directories(manifold PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/manifold/3rd_party/libigl/include)
target_link_libraries(manifold PRIVATE Threads::Threads)

add_executable(simplify simplify/src.cmd/Main.cpp)
target_link_libraries(simplify PRIVATE Threads::Threads)
