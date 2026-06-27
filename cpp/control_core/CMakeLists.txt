pybind11_add_module(control_core
    control_core.cpp
    line_sensor.cpp
    lateral_controller.cpp
    speed_controller.cpp
    vw_controller.cpp
)

target_link_libraries(control_core PRIVATE micromouse_shared)
target_include_directories(control_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Install alongside Python package
install(TARGETS control_core DESTINATION .)
