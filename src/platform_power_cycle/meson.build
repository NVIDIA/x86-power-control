platform_power_cycle_lib = static_library(
    'platform-power-cycle',
    'platform_power_cycle.cpp',
    dependencies: [sdbusplus, phosphordbusinterfaces],
)

platform_power_cycle_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: platform_power_cycle_lib,
    dependencies: [sdbusplus, phosphordbusinterfaces],
)

executable(
    'nvidia-platform-power-cycled',
    'main.cpp',
    dependencies: [platform_power_cycle_dep, boost],
    install: true,
    install_dir: get_option('bindir'),
)

gtest = dependency('gtest', main: true, required: false)
if gtest.found()
    test_executable = executable(
        'test-platform-power-cycle',
        'test_supported_types.cpp',
        dependencies: [platform_power_cycle_dep, gtest],
    )
    test('platform-power-cycle', test_executable)
endif
