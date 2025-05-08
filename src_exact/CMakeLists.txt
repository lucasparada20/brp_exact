cmake_minimum_required(VERSION 3.2)
project(ExecExact)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Hard code the path to the CPLEX directory
set(CPLEX_DIR "/home/lucasparada20/CPLEX_Studio2211")
message("Using CPLEX_DIR = ${CPLEX_DIR}")

# Include directories for CPLEX
set(CPLEX_INCLUDE_DIRS 
    ${CPLEX_DIR}/cplex/include
    ${CPLEX_DIR}/concert/include
)

# Link directories for CPLEX
set(CPLEX_LIBRARY_DIRS 
    ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic
    ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic
)

# Flags for CPLEX
set(CPLEX_FLAGS ilocplex concert cplex m pthread dl)

message("CPLEX_INCLUDE_DIRS = ${CPLEX_INCLUDE_DIRS}")
message("CPLEX_LIBRARY_DIRS = ${CPLEX_LIBRARY_DIRS}")
message("CPLEX_FLAGS = ${CPLEX_FLAGS}")

# Create a shared interface target for CPLEX 
add_library(CPLEX INTERFACE)
target_include_directories(CPLEX INTERFACE ${CPLEX_INCLUDE_DIRS})
target_link_directories(CPLEX INTERFACE ${CPLEX_LIBRARY_DIRS})
target_link_libraries(CPLEX INTERFACE ${CPLEX_FLAGS})

# Check that all required paths and libraries exist
foreach(dir IN LISTS CPLEX_INCLUDE_DIRS CPLEX_LIBRARY_DIRS)
    if(NOT EXISTS ${dir})
        message(FATAL_ERROR "Required CPLEX path not found: ${dir}")
    endif()
endforeach()

# CVRPSEP sources
set(CVRPSEP_SRC_DIR "/home/lucasparada20/work/brp/src_cvrpsep")
# The line below is equivalent to set(Sources ... all .cpp files ...)
file(GLOB CVRPSEP_SOURCES "${CVRPSEP_SRC_DIR}/*.cpp")
include_directories(${CVRPSEP_SRC_DIR})

# Source files
# All source files see the cplex headers
set(SOURCES
	main_ex_brp.cpp
	Parameters.cpp
	LoadBRP.cpp
	ExactBrp.cpp
	ExactBrpGraph.cpp
	Network.cpp
	ExactBrpCallBacks.cpp
	RouteFeasibility.cpp
	DriverCountBppLb.cpp
	CostFunctionSBRP.cpp
	InsRmvMethodSBRP.cpp
	SequentialInsertionSBRP.cpp
	AlnsBRP.cpp
	BestSolutionList.cpp
	CostFunctionBRP.cpp
	SequentialInsertionBRP.cpp
	InsRmvMethodBRP.cpp
	RandomRemoveBRP.cpp
	RelatednessRemoveBRP.cpp	
)

# Target: exec_exact
option(USE_CVRPSEP "Use CVRPSEP-based ExactBrpSep implementation" OFF)

if(USE_CVRPSEP)
	add_executable(exec_exact ExactBrpSepCvrpsep.cpp ${SOURCES} ${CVRPSEP_SOURCES})
	target_compile_definitions(exec_exact PRIVATE USE_CVRPSEP)
else()
	add_executable(exec_exact ExactBrpSepUser.cpp ${SOURCES})
	target_compile_options(exec_exact PRIVATE) 
endif()

target_link_libraries(exec_exact PRIVATE CPLEX)

# Usage message when typing make in the build directory
add_custom_target(usage
    COMMAND ${CMAKE_COMMAND} -E echo "Call: ./exec_exact to see how to use the executable"
)

