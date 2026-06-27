pybind11_add_module(localize_core
    localize_core.cpp
    imu_processor.cpp
    odometry.cpp
    velocity_kalman.cpp
    slip_detector.cpp
    pipeline_localizer.cpp
)

target_link_libraries(localize_core PRIVATE micromouse_shared)
target_include_directories(localize_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Install alongside Python package
install(TARGETS localize_core DESTINATION .)
