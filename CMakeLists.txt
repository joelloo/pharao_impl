cmake_minimum_required(VERSION 3.5)

#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE} CACHE STRING "" FORCE)

project(so_pharao)

# ============================================================================== #
#					DEFINE FUNCTION TO CREATE FILTERS IN VS                      #
# ============================================================================== #
function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else()
            set(_source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)


# ============================================================================== #
#                          BUILD_TOOL CONFIGURATIONS                             #
# ============================================================================== #
#SET(BUILD_TOOL cmake) # !! For CMake
SET(BUILD_TOOL ament_cmake) # !! For ament_cmake

# ============================================================================== #
#                          COMPILATION FLAGS CONFIG                              #
# ============================================================================== #
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

IF(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
  # under windows, explicitly enable math defines like M_PI
  # allow larget objects than default
  # enable parallel compilation
  ADD_DEFINITIONS("-D _USE_MATH_DEFINES /MP /fp:fast /INCREMENTAL")
ENDIF()


# ============================================================================== #
#                                 Find Packages                                  #
# ============================================================================== #

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(Boost_ARCHITECTURE "-x64")

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)

find_library(FFTW_LIBRARY NAMES fftw3 fftw)
set(FFTW_LIBRARIES "${FFTW_LIBRARY}")

find_library(FFTW_libm_LIBRARY NAMES m)
list(APPEND FFTW_LIBRARIES "${FFTW_libm_LIBRARY}")

IF(BUILD_TOOL MATCHES "ament_cmake")   
   find_package(ament_cmake_autovision REQUIRED)
   find_package(cereal_ament REQUIRED)
   find_package(ceres_ament REQUIRED)
   find_package(class_loader REQUIRED)
   find_package(autovision_common REQUIRED)
   find_package(image_converter REQUIRED)
   find_package(autovision_msgs REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclcpp_components REQUIRED)
   find_package(std_msgs REQUIRED)
   find_package(geometry_msgs REQUIRED)
   find_package(rmw_connext_qos REQUIRED)
ENDIF()

# ============================================================================== #
#                        Add package include/lib path                            # 
# ============================================================================== #
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${cereal_ament_INCLUDE_DIRS})
include_directories(${ceres_ament_INCLUDE_DIRS})
include_directories(${fftw_INCLUDE_DIRS})

add_definitions(${DEFINITIONS})

IF(BUILD_TOOL MATCHES "ament_cmake")
    # ament_cmake for ros2
    LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib/)
ENDIF()

include_directories(${PROJECT_SOURCE_DIR}/include)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

#############################
# Specify the project files #
#############################

SET(pharao_sources
src/utils/Parameter.cpp
src/utils/ParameterList.cpp
src/utils/Stopwatch.cpp
src/registration/logpolar_fft/PhaseCorrelation.cpp
src/registration/logpolar_fft/LogPolarFFT.cpp
src/registration/imreg_dft/image_dft.cpp
src/registration/imreg_dft/image_registration.cpp
src/registration/imreg_dft/image_transforms.cpp
src/registration/ScanMatching.cpp
src/Odometry.cpp
src/main.cpp
)

SET(pharao_headers
include/Odometry.hpp
include/PoseGraphResidual.hpp
include/entity/Frame.hpp
include/entity/FrameBuffer.hpp
include/entity/Pose2D.hpp
include/registration/imreg_dft/image_dft.h
include/registration/imreg_dft/image_registration.h
include/registration/imreg_dft/image_transforms.h
include/registration/logpolar_fft/LogPolarFFT.hpp
include/registration/logpolar_fft/PhaseCorrelation.hpp
include/registration/ScanMatching.hpp
include/utils/Angle.hpp
include/utils/Parameter.h
include/utils/ParameterList.h
include/utils/Stopwatch.h
include/utils/parsing/PoseDataParser.hpp
include/utils/parsing/RadarDataParser.hpp
include/utils/Visualiser.hpp
)

IF(BUILD_TOOL MATCHES "ament_cmake")
	add_executable(pharao ${pharao_sources} ${pharao_headers})
	ament_target_dependencies(pharao
		"rclcpp"
		"std_msgs"
		"geometry_msgs"
		"autovision_msgs"
		"autovision_common"
		"rmw_connext_qos"
		"Boost"
    "ceres_ament"
		"OpenCV"
		"VTK")

	target_link_libraries(pharao ${FFTW_LIBRARY}/libfftw3-3.lib)

	add_executable (test_ceres src/test_ceres.cpp src/utils/Stopwatch.cpp)
	ament_target_dependencies(test_ceres "ceres_ament" "OpenCV")

	assign_source_group(${pharao_sources})
	assign_source_group(${pharao_headers})

	ament_package()

	install(
	  DIRECTORY configuration_files
	  DESTINATION share/${PROJECT_NAME}
	)

	install(
	  TARGETS pharao test_ceres
	  DESTINATION lib/${PROJECT_NAME}
	)


ENDIF()
