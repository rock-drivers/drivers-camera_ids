# for ueye api version 

#add_executable(api_version api_version)
#target_link_libraries(api_version ueye_api)

execute_process(COMMAND g++ ${CMAKE_SOURCE_DIR}/configure/api_version.cpp 
    -o api_version -lueye_api)
execute_process(COMMAND ${CMAKE_BINARY_DIR}/api_version 
    OUTPUT_VARIABLE API_VERSION )
message(STATUS "ueye api version: ${API_VERSION}")
add_definitions(-DAPI_VERSION=${API_VERSION})
