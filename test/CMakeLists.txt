include_directories(${CMAKE_SOURCE_DIR}/src)

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
