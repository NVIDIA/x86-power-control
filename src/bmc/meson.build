# BMC state manager subdirectory
# Inherits dependencies and configuration from parent meson.build

executable(
    'bmc-state-manager',
    'bmc_state_manager.cpp',
    'bmc_state_manager_main.cpp',
    'utils.cpp',
    cpp_args: cpp_args,
    dependencies: [
        libgpiod,
        phosphordbusinterfaces,
        phosphorlogging,
        sdbusplus,
        sdeventplus,
    ],
    include_directories: root_build_inc,
    install: true,
)


systemd = dependency('systemd')
if systemd.found()
    install_data(
        'xyz.openbmc_project.State.BMC.service',
        install_dir: systemd.get_variable('systemdsystemunitdir'),
    )
endif

# Install D-Bus based powerctrl script if enabled
# When disabled, platform-specific scripts can be installed via bbappend
if get_option('install-powerctrl').enabled()
    install_data(
        'powerctrl.sh',
        install_dir: get_option('bindir'),
        install_mode: 'rwxr-xr-x',
    )
endif
