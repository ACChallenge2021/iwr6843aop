
add_library(${CMAKE_PROJECT_NAME}	
			${CMAKE_CURRENT_SOURCE_DIR}/iwr6843aop.cpp
			${CMAKE_CURRENT_SOURCE_DIR}/DataHelper.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
							PUBLIC
								$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/public_include>
							PRIVATE 
							 	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC serial)


add_executable(${CMAKE_PROJECT_NAME}_unit_test ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/main.cpp
												${CMAKE_CURRENT_SOURCE_DIR}/DataHelper.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_unit_test 
							PRIVATE 
								${CMAKE_CURRENT_SOURCE_DIR}
								${CMAKE_CURRENT_SOURCE_DIR}/public_include
)

install(TARGETS ${PROJECT_NAME}_unit_test DESTINATION "bin" CONFIGURATIONS Release Debug
                        COMPONENT "${PROJECT_NAME}-bin")
				