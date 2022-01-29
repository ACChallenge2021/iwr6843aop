
add_library(${CMAKE_PROJECT_NAME}	
			${CMAKE_CURRENT_SOURCE_DIR}/iwr6843aop.cpp
#			${CMAKE_CURRENT_SOURCE_DIR}/DataUARTHandler.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
							PUBLIC
								$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/public_include>
							PRIVATE 
							 	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC serial)
							
						