cmake_minimum_required(VERSION 2.8)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

# Here we tell cmake that we want to make a project called "rrt", which will 
# automatically create a variable called PROJECT_NAME (set to "rrt"). We then 
# tell CMake the version number of the project.
project(iago)
set(${PROJECT_NAME}_VERSION 0.0.1)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS src/iago_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

enable_testing()
add_test(NAME ${PROJECT_NAME}
	COMMAND nosetests --with-doctest --with-coverage --cover-html
  --cover-html-dir=${PROJECT_NAME}_coverage_html --cover-xml
	--cover-xml-file=${PROJECT_NAME}_coverage.xml --cover-package=metis
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

