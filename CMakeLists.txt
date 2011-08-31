##############################
#
# WELCOME
#
# This is the main cmake configuration file
# associate to your project
# You can define here specific compilation options
#
# More about cmake : www.cmake.org
#
# Using cmake command line :
# 1. Create a build directory : mkdir build
# 2. Configure cmake project : cd build; cmake ..
#   You can also generate project with your favorite IDE :
#   Example : Under Mac osx use "cmake .. -G XCode"
#   see cmake help for more information
# 3. Edit project option : ccmake .
# 4. Build it : make

############################ PROJECT NAME
PROJECT(seu-spark)

############################ VERBOSITY
SET(CMAKE_VERBOSE_MAKEFILE on)

############################ VERSION
# Check cMake version
CMAKE_MINIMUM_REQUIRED( VERSION 2.6.0 )

############################ COLORIZE MAKEFILE
# Colorized output
SET( CMAKE_COLOR_MAKEFILE TRUE )

############################ CHOOSE COMPILE MODE
SET(CMAKE_CXXFLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXXFLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")
SET(CMAKE_BUILD_TYPE "Release")
############################ CMAKE PACKAGE DIRECTORY
# Set cmakeModules folder
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

############################ FIND PACKAGE
# Libraries specifics. Define here cmake packages
# usefull to your project

#FIND_PACKAGE(PythonLibs 2.6 REQUIRED)
FIND_PACKAGE(Boost REQUIRED thread python)
SET(Boost_USE_STATIC_LIBS ON)

############################ SUBDIRECTORIES INCLUSION
ADD_SUBDIRECTORY( src )
ADD_SUBDIRECTORY( doc )


