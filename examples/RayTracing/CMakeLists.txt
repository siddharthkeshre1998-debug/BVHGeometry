add_executable(
    RayTracingExample
    main.cpp
)

target_link_libraries(
    RayTracingExample
    PRIVATE
        GeometryKernel
)
