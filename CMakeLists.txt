cmake_policy(SET CMP0048 NEW)

project(AKDController VERSION 0.1)
cmake_minimum_required(VERSION 3.9)


#Dependency
add_subdirectory(extern/SOEM)

#Library
add_library(AKD_ecat STATIC src/AKDEcatController.cpp)
target_link_libraries(AKD_ecat PUBLIC soem pthread)
target_include_directories(AKD_ecat PUBLIC include)


#Tests
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  add_subdirectory(tests/)
else()
  message(STATUS "AKD_Ecat: not building tests when built as dependency")
endif()




