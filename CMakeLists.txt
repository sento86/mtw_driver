cmake_minimum_required(VERSION 2.4.6)
project(mtw_driver)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	sensor_msgs
	geometry_msgs
	message_generation
	tf
)

add_message_files(
			FILES
			QuaternionStampedMultiarray.msg
			TransformStampedMultiarray.msg
			ImuStampedMultiarray.msg
)

generate_messages(
			DEPENDENCIES
			std_msgs
			geometry_msgs
			sensor_msgs
)


catkin_package(
			INCLUDE_DIRS include			
			#LIBRARIES ${PROJECT_NAME}
			CATKIN_DEPENDS roscpp tf message_runtime
			DEPENDS message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  #/usr/local/xsens/include
  /home/idf/libraries/Xsens-MTi-4.7/include
)

#link_directories( /usr/local/xsens/lib )
link_directories( /home/idf/libraries/Xsens-MTi-4.7/lib )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(mtw_driver_node src/main_mtw.cpp src/conio.c)
target_link_libraries(mtw_driver_node m xsensdeviceapi xstypes pthread ${catkin_LIBRARIES})
add_dependencies(mtw_driver_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(mtw_driver_node_ros src/main_mtw_ros.cpp src/conio.c)
target_link_libraries(mtw_driver_node_ros m xsensdeviceapi xstypes pthread ${catkin_LIBRARIES})
add_dependencies(mtw_driver_node_ros ${PROJECT_NAME}_generate_messages_cpp)

##################################

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
