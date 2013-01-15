rock_library(camera_ids
    SOURCES CamIds.cpp
    HEADERS CamIds.h
    DEPS_PKGCONFIG base-types camera_interface base-lib)

target_link_libraries(${PROJECT_NAME} ueye_api)
