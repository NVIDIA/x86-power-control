project(
    'x86-power-control',
    'cpp',
    default_options: ['warning_level=3', 'cpp_std=c++23'],
    license: 'Apache-2.0',
    version: '0.1',
    meson_version: '>=1.1.1',
)

cpp = meson.get_compiler('cpp')
fs = import('fs')

add_project_arguments(
    '-Wno-psabi',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    language: 'cpp',
)

cpp_args = []

if get_option('chassis-system-reset').allowed()
    cpp_args += '-DCHASSIS_SYSTEM_RESET'
endif
if get_option('use-plt-rst').allowed()
    cpp_args += '-DUSE_PLT_RST'
endif
if get_option('use-acboot').allowed()
    cpp_args += '-DUSE_ACBOOT'
endif
if get_option('ignore-soft-resets-during-post').allowed()
    cpp_args += '-DIGNORE_SOFT_RESETS_DURING_POST'
endif
if get_option('button-passthrough').allowed()
    cpp_args += '-DUSE_BUTTON_PASSTHROUGH'
endif
if get_option('enable-bmc-poweroff').allowed()
    cpp_args += '-DENABLE_BMC_POWEROFF'
endif
if get_option('cpu-reset-early-assert-war').allowed()
    cpp_args += '-DCPU_RESET_EARLY_ASSERT_WAR'
endif

# IOX paths are no longer compile-time options — each platform's deployed
# power-config-host0.json (shipped by its BSP meta layer) declares its IOX
# paths in the "resources" section, validated at runtime.
conf_data = configuration_data()

conf_data.set_quoted('OBMC_STANDBY_TARGET', get_option('obmc-standby-target'))
conf_data.set_quoted(
    'CHASSIS_LOST_POWER_FILE',
    '/run/openbmc/chassis@{}-lost-power',
)


configure_file(output: 'config.h', configuration: conf_data)

# Include directory for generated config.h (available to subdirectories)
root_build_inc = include_directories('.')

libgpiod = dependency('libgpiod', default_options: ['bindings=cxx'])
libgpiodcxx = dependency('libgpiodcxx')
libsystemd = dependency('libsystemd')
nlohmann_json_dep = dependency('nlohmann_json', include_type: 'system')
phosphordbusinterfaces = dependency('phosphor-dbus-interfaces')
phosphorlogging = dependency('phosphor-logging')
sdbusplus = dependency('sdbusplus')
sdeventplus = dependency('sdeventplus')
boost = dependency('boost')

platform_power_cycle_option = get_option('platform-power-cycle-service')
platform_power_cycle_contract = false
if not platform_power_cycle_option.disabled()
    if phosphordbusinterfaces.type_name() == 'internal'
        pdi_yamldir = phosphordbusinterfaces.get_variable(internal: 'yamldir')
        platform_power_cycle_contract = fs.exists(
            pdi_yamldir / 'com/nvidia/Control/Platform/PowerCycle.interface.yaml',
        )
    else
        platform_power_cycle_contract = cpp.has_header(
            'com/nvidia/Control/Platform/PowerCycle/server.hpp',
            dependencies: phosphordbusinterfaces,
        )
    endif
endif
if platform_power_cycle_option.enabled() and not platform_power_cycle_contract
    error(
        'platform-power-cycle-service requires the PlatformPowerCycle D-Bus contract',
    )
endif
build_platform_power_cycle = not platform_power_cycle_option.disabled() and platform_power_cycle_contract
deps = [
    libgpiodcxx,
    libsystemd,
    sdbusplus,
    phosphorlogging,
    nlohmann_json_dep,
    boost,
]

executable(
    'x86-power-control',
    'src/power_control.cpp',
    'src/power_control_base.cpp',
    'src/vr_power_control.cpp',
    'src/platform/nvl72/nvl72_power_control.cpp',
    'src/platform/c2/c2_power_control.cpp',
    'src/platform/vr_nvl8/vr_nvl8_power_control.cpp',
    'src/platform/vc256/vc256_power_control.cpp',
    'src/platform/gnr/gnr_power_control.cpp',
    include_directories: include_directories('src'),
    cpp_args: cpp_args,
    dependencies: deps,
    install: true,
    install_dir: get_option('bindir'),
)

systemd = dependency('systemd')
if systemd.found()
    if build_platform_power_cycle
        platform_power_cycle_service_conf = configuration_data()
        platform_power_cycle_service_conf.set(
            'BINDIR',
            join_paths(get_option('prefix'), get_option('bindir')),
        )
        configure_file(
            input: 'service_files/com.nvidia.Control.Platform.PowerCycle.service.in',
            output: 'com.nvidia.Control.Platform.PowerCycle.service',
            configuration: platform_power_cycle_service_conf,
            install: true,
            install_dir: systemd.get_variable('systemdsystemunitdir'),
        )
    endif
    install_data(
        'service_files/chassis-system-reset.service',
        'service_files/chassis-system-reset.target',
        'service_files/xyz.openbmc_project.Chassis.Control.Power@.service',
        install_dir: systemd.get_variable('systemdsystemunitdir'),
    )
endif

install_data(
    'config/power-config-host0.json',
    install_dir: '/usr/share/x86-power-control/',
)

if build_platform_power_cycle
    subdir('src/platform_power_cycle')
endif
subdir('src/bmc')
