# for ueye
message(STATUS "-DAPI_VERSION=${API_VERSION}")
add_definitions(-DAPI_VERSION=${API_VERSION})

rock_library(camera_ids
    SOURCES CamIds.cpp
    HEADERS CamIds.h
    DEPS_PKGCONFIG base-types camera_interface base-lib)
#set(module_name ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ueye_api)

