
add_executable(${CMAKE_PROJECT_NAME}_unit_test ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_unit_test 
							PRIVATE 
							${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${CMAKE_PROJECT_NAME}_unit_test PRIVATE iwr6843aop)

install(TARGETS ${PROJECT_NAME}_test DESTINATION "bin" CONFIGURATIONS Release Debug
                        COMPONENT "${PROJECT_NAME}-bin")
