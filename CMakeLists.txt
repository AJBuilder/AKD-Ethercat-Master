cmake_policy(SET CMP0048 NEW)

project(AKDController VERSION 0.1)
cmake_minimum_required(VERSION 3.9)


# Check to make sure submodule was initialized.
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/SOEM/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()



#Dependency
add_subdirectory(extern/SOEM)


add_library(AKD_ecat STATIC src/AKDEcatController.cpp)
#Library
#if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
#    add_library(AKD_ecat STATIC src/AKDEcatController.cpp)
#else()
#    add_library(AKD_ecat OBJECT src/AKDEcatController.cpp)
#    message(STATUS "AKD_Ecat: not creating a library archive (libAKD-ecat.a) when build as dependecy.")
#endif()

target_link_libraries(AKD_ecat PUBLIC soem pthread)
target_include_directories(AKD_ecat PUBLIC include)


#Tests
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_executable(simpleMtrCtrl tests/simpleMtrCtrl.cpp)
    add_executable(simpleMtrCtrlx2 tests/simpleMtrCtrlx2.cpp)

    target_link_libraries(simpleMtrCtrl PUBLIC AKD_ecat)
    target_link_libraries(simpleMtrCtrlx2 PUBLIC AKD_ecat)

    install(TARGETS simpleMtrCtrl DESTINATION tests/)
    install(TARGETS simpleMtrCtrlx2 DESTINATION tests/)
else()
  message(STATUS "AKD_Ecat: not building tests when built as dependency")
endif()


set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)

install( FILES include/AKDEcatController.h
    DESTINATION include/
)
install( TARGETS AKD_ecat
    DESTINATION lib/
)


