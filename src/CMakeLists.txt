rock_library(camera_ids
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(camera_ids_bin Main.cpp
    DEPS camera_ids)

