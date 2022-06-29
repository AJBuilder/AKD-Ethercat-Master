project(EtherCATBench)
cmake_minimum_required(VERSION 3.9)
#cmake_policy(SET CMP0015 NEW)




add_executable(simpleMtrCtrl src/simpleMtrCtrl.cpp)
add_library(AKD OBJECT src/AKDEcatController.cpp src/AKDEcatController.h)


target_include_directories(simpleMtrCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_include_directories(AKD PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_link_directories(simpleMtrCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/lib)



target_link_libraries(simpleMtrCtrl PUBLIC libsoem.a pthread AKD)

install(TARGETS simpleMtrCtrl DESTINATION build)