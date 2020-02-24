cmake_minimum_required(VERSION 3.0)
project(dec_pomdp_algorithm)

if(BUILD_TEST)
	include(CTest)

	if (CMAKE_VERSION VERSION_LESS 3.2)
	    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
	else()
	    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
	endif()

	include(DownloadProject.cmake)
	download_project(PROJ                googletest
	                 GIT_REPOSITORY      https://github.com/google/googletest.git
	                 GIT_TAG             master
	                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
	)

	# Prevent GoogleTest from overriding our compiler/linker options
	# when building with Visual Studio
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

	add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})


	# When using CMake 2.8.11 or later, header path dependencies
	# are automatically added to the gtest and gmock targets.
	# For earlier CMake versions, we have to explicitly add the
	# required directories to the header search path ourselves.
	if (CMAKE_VERSION VERSION_LESS 2.8.11)
	    include_directories("${gtest_SOURCE_DIR}/include"
	                        "${gmock_SOURCE_DIR}/include")
	endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
	dec_pomdp_msgs
)

catkin_package(
)

## MADP toolbox
if(MADPPATH)
	set(MADP_INCLUDE_DIRECTORIES ${MADPPATH}/include)
	set(MADP_LIBRARY_PATH ${MADPPATH}/lib)
else()
	include(ExternalProject)
	set(EXTERNALS_DIR ${CMAKE_CURRENT_BINARY_DIR}/external)
	ExternalProject_Add(project_madp
	    URL               https://github.com/MADPToolbox/MADP/archive/v0.4.1.tar.gz
	    URL_HASH          MD5=b385c9d4c2b478c0b0611774ee7f26bf
	    PREFIX            ${EXTERNALS_DIR}/madp
	    BUILD_IN_SOURCE	  1
	    CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
	    BUILD_COMMAND     make -j12
	    INSTALL_COMMAND   make install
	)

	ExternalProject_Get_Property(project_madp INSTALL_DIR)
	set(MADP_INCLUDE_DIRECTORIES ${INSTALL_DIR}/include)
	set(MADP_LIBRARY_PATH ${INSTALL_DIR}/lib)

	message(STATUS "MADP_INCLUDE_DIRECTORIES: ${MADP_INCLUDE_DIRECTORIES}")

	add_library(MADP STATIC IMPORTED)
	set_property(TARGET MADP PROPERTY IMPORTED_LOCATION ${MADP_LIBRARY_PATH}/libMADP-0.4.1.so)
	add_dependencies(MADP project_madp)

endif()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include
					${EIGEN3_INCLUDE_DIR}
					SYSTEM ${MADP_INCLUDE_DIRECTORIES})
link_directories(${MADP_LIBRARY_PATH})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/dec_pomdp_algorithm.cpp
# )




if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

add_definitions(
    -Wfatal-errors
    )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(catkin REQUIRED)

FIND_PACKAGE(Boost 1.58 COMPONENTS program_options regex graph REQUIRED)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

# Add warnings and errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")


add_library(MADPWrapper src/MADPWrapper.cpp)
target_link_libraries(MADPWrapper MADP Threads::Threads)

add_library(Base
					src/BeliefUtilities.cpp
					src/DecPOMDPUtilities.cpp
					src/History.cpp
					src/HistoryData.cpp
					src/HistoryCacheUtils.cpp
					src/PolicyGraph.cpp
					src/LocalPolicyHistory.cpp
					src/JointPolicy.cpp
					src/JointPolicyHistory.cpp
					src/JointPolicyHistories.cpp
					src/JointPolicyUtilities.cpp
					src/PolicyInitialization.cpp
					src/PolicyGraphTraversal.cpp
					src/MADPWrapperUtils.cpp
					src/ValueFunction.cpp
)

target_link_libraries(Base MADPWrapper)

add_library(NPGIPlanner
					src/BackwardPass.cpp
					src/ValueUtilities.cpp

					)

target_link_libraries(NPGIPlanner Base)


add_library(NPGIParticles
				src/ParticleUtilities.cpp)

add_executable(${PROJECT_NAME}
				src/DecPOMDPPublisher.cpp
				src/GraphSensingProblem.cpp
				)

add_executable(solve_graphsensing
				src/solve_graphsensing.cpp
				src/GraphSensingProblem.cpp
				)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} MADPWrapper Base NPGIPlanner NPGIParticles ${Boost_LIBRARIES})
target_link_libraries(solve_graphsensing Base NPGIParticles ${Boost_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/dec_pomdp_algorithm_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dec_pomdp_algorithm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
