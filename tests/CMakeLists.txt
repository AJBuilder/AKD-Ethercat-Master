add_executable(simpleMtrCtrl simpleMtrCtrl.cpp)
add_executable(simpleMtrCtrlx2 simpleMtrCtrlx2.cpp)

target_link_libraries(simpleMtrCtrl PUBLIC AKD_ecat)
target_link_libraries(simpleMtrCtrlx2 PUBLIC AKD_ecat)


install(TARGETS simpleMtrCtrl DESTINATION ../build/tests/)
install(TARGETS simpleMtrCtrlx2 DESTINATION ../build/tests/)