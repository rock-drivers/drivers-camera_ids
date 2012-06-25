# for ueye
include_directories(/usr/local/include)
link_directories (/use/local/lib)

rock_library(camera_ids
    SOURCES CamIds.cpp
    HEADERS CamIds.h
    DEPS_PKGCONFIG base-types camera_interface base-lib)
#set(module_name ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ueye_api)

pkg_check_modules(OPENCV QUIET opencv)
if(OPENCV_FOUND)
    find_package(Boost REQUIRED COMPONENTS program_options)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})

    rock_executable(test_bin test_cam.cpp
        DEPS camera_ids
        DEPS_PKGCONFIG opencv
        DEPS_CMAKE Boost)

endif(OPENCV_FOUND)
