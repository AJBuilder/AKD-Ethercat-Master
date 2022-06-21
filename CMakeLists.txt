project(EtherCATBench)
cmake_minimum_required(VERSION 3.9)
#cmake_policy(SET CMP0015 NEW)




add_executable(simpleMtrCtrl src/simpleMtrCtrl.cpp)
add_library(DS402 OBJECT src/DS402EcatController.cpp src/DS402EcatController.h)


target_include_directories(simpleMtrCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_include_directories(DS402 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_link_directories(simpleMtrCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/lib)



target_link_libraries(simpleMtrCtrl PUBLIC libsoem.a pthread DS402)

install(TARGETS simpleMtrCtrl DESTINATION build)