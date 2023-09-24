project(csf_filter)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_PREFIX_PATH "/home/gj/c++_study/test/CSF_test/CSF/CSF/install/")


find_package(OpenMP)
find_library(CSF_LIBRARY NAMES CSF)
find_path(CSF_INCLUDE_DIR CSF.h)

find_package(PCL REQUIRED)


if(NOT CSF_LIBRARY OR NOT CSF_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find CSF library or header")
endif()

include_directories(
    "/home/gj/c++_study/test/CSF_test/CSF/CSF/install/include"
)



include_directories(

    ${PCL_INCLUDE_DIRS}
    # ${Boost_INCLUDE_DIRS}
)




link_directories(
    ${PCL_LINBRARY_DIRS}
    
)

add_executable(csf_filter csf_filter.cpp)
target_link_libraries(csf_filter ${CSF_LIBRARY} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)


