# Collect file information
file(GLOB network_SRC "*.cpp")
file(GLOB network_HEADER "*.h")

add_library(network ${network_SRC} ${network_HEADER})
target_include_directories(network PUBLIC ${PROJECT_SOURCE_DIR}/src)

# Add Gurobi
find_package(GUROBI REQUIRED)
target_link_libraries(network PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
target_link_libraries(network PUBLIC ${GUROBI_LIBRARY})
target_include_directories(network PRIVATE "${GUROBI_INCLUDE_DIRS}")

add_subdirectory(global)
add_subdirectory(detailed)
