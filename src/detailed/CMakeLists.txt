# Collect file information
file(GLOB detailed_SRC "*.cpp")
file(GLOB detailed_HEADER "*.h")

add_library(detailed ${detailed_SRC} ${detailed_HEADER})
target_include_directories(detailed PUBLIC ${PROJECT_SOURCE_DIR}/src/detailed)

# Add Gurobi
find_package(GUROBI REQUIRED)
target_link_libraries(detailed PUBLIC optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
target_link_libraries(detailed PUBLIC ${GUROBI_LIBRARY})
target_include_directories(detailed PRIVATE "${GUROBI_INCLUDE_DIRS}")

