zephyr_library_named(hall_trackball)

zephyr_library_sources(hall_trackball.c)
target_include_directories(hall_trackball PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Device tree bindings (if module)
zephyr_include_directories_ifdef(CONFIG_HALL_TRACKBALL_DRIVER ${CMAKE_CURRENT_SOURCE_DIR}/../dts/bindings)