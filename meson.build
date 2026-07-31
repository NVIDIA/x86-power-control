project(
    'x86-power-control',
    'cpp',
    default_options: ['warning_level=3', 'cpp_std=c++23'],
    license: 'Apache-2.0',
    version: '0.1',
    meson_version: '>=1.1.1',
)

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
    include_directories: include_directories('src'),
    cpp_args: cpp_args,
    dependencies: deps,
    install: true,
    install_dir: get_option('bindir'),
)

systemd = dependency('systemd')
if systemd.found()
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

subdir('src/bmc')
