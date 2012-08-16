# for ueye api version 

#add_executable(api_version api_version)
#target_link_libraries(api_version ueye_api)

execute_process(COMMAND g++ ${CMAKE_SOURCE_DIR}/configure/api_version.cpp 
    -o api_version -lueye_api)
execute_process(COMMAND ${CMAKE_BINARY_DIR}/api_version 
    OUTPUT_VARIABLE API_VERSION_LOCAL )
message(STATUS "ueye api version: ${API_VERSION_LOCAL}")
set(API_VERSION ${API_VERSION_LOCAL} PARENT_SCOPE)
