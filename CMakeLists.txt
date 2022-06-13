project(EtherCATBench)
cmake_minimum_required(VERSION 3.9)
#cmake_policy(SET CMP0015 NEW)




add_executable(periodTest src/periodTest.c)
add_executable(simpleMtrCtrl src/simpleMtrCtrl.c)
add_executable(dcTest src/dcTest.c)


target_include_directories(periodTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_link_directories(periodTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/lib)

target_include_directories(simpleMtrCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_link_directories(simpleMtrCtrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/lib)

target_include_directories(dcTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/include/soem)
target_link_directories(dcTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/install/lib)


target_link_libraries(periodTest PUBLIC libsoem.a pthread)
target_link_libraries(simpleMtrCtrl PUBLIC libsoem.a pthread)
target_link_libraries(dcTest PUBLIC libsoem.a pthread)


install(TARGETS periodTest DESTINATION build)
install(TARGETS simpleMtrCtrl DESTINATION build)
install(TARGETS dcTest DESTINATION build)