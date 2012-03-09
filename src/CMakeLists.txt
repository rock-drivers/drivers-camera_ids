# for ueye
include_directories(/usr/local/include)
link_directories (/use/local/lib)

rock_library(camera_ids
    SOURCES CamIds.cpp
    HEADERS CamIds.h
    DEPS_PKGCONFIG base-types camera_interface base-lib)

#set(module_name ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ueye_api)

rock_executable(test_bin test.cpp
    DEPS camera_ids
    DEPS_PKGCONFIG opencv)

