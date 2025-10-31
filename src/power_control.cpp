/*
// Copyright (c) 2018-2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#include "config.h"
#include "power_control.hpp"

#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <string_view>

namespace power_control
{
static boost::asio::io_context io;
std::shared_ptr<sdbusplus::asio::connection> conn;
PersistentState appState;
PowerRestoreController powerRestore(io);

static std::string node = "0";
static const std::string appName = "power-control";\

enum class PowerAction {
    NONE,
    POWER_ON,
    FORCE_OFF,
    GRACE_OFF,
    POWER_CYCLE,
    SYSTEM_RESET,
    HOST_INITIATED_SHUTDOWN,
};

struct BoardPresence {
    bool c2_pdb = false;
    bool nvl144_pdb = false;
    bool board0 = false;
    bool board1 = false;
};

struct Context {
    PowerAction action = PowerAction::NONE;
    std::string target_state = "HostOff";
    BoardPresence presence;
};

static Context powerContext;

enum class DbusConfigType
{
    name = 1,
    path,
    interface,
    property
};

// Mandatory config parameters for dbus inputs
boost::container::flat_map<DbusConfigType, std::string> dbusParams = {
    {DbusConfigType::name, "DbusName"},
    {DbusConfigType::path, "Path"},
    {DbusConfigType::interface, "Interface"},
    {DbusConfigType::property, "Property"}};

enum class ConfigType
{
    GPIO = 1,
    DBUS
};

struct ConfigData
{
    std::string name;
    std::string lineName;
    std::string dbusName;
    std::string path;
    std::string interface;
    std::optional<std::regex> matchRegex;
    bool polarity;
    ConfigType type;
};

static ConfigData powerOutConfig;
static ConfigData powerOkConfig;
static ConfigData resetOutConfig;
static ConfigData nmiOutConfig;
static ConfigData sioPwrGoodConfig;
static ConfigData sioOnControlConfig;
static ConfigData sioS5Config;
static ConfigData postCompleteConfig;
static ConfigData powerButtonConfig;
static ConfigData resetButtonConfig;
static ConfigData idButtonConfig;
static ConfigData nmiButtonConfig;
static ConfigData slotPowerConfig;
static ConfigData hpmStbyEnConfig;
static ConfigData nvl144pdbMainPowerEnableConfig;
static ConfigData nvl144pdbMainPowerOkConfig;
static ConfigData c2pdbPSUPowerEnableConfig;
static ConfigData c2pdbPSUPowerOkConfig;
static ConfigData c2pdb_12V_HPMEnableConfig;
static ConfigData c2pdb_12V_GPU1EnableConfig;
static ConfigData c2pdb_12V_GPU2EnableConfig;
static ConfigData c2pdb_12V_AICEnableConfig;
static ConfigData usbPowerEnableConfig;
static ConfigData board0RunPowerPGConfig;
static ConfigData board1RunPowerPGConfig;
static ConfigData board0RunPowerEnableConfig;
static ConfigData board1RunPowerEnableConfig;
static ConfigData board0PreSystemResetConfig;
static ConfigData board1PreSystemResetConfig;
static ConfigData cpuResetIndicatorConfig;
static ConfigData board0CpuShutdownForceConfig;
static ConfigData board1CpuShutdownForceConfig;
static ConfigData board0CpuShutdownRequestConfig;
static ConfigData board1CpuShutdownRequestConfig;
static ConfigData board0CpuShutdownOkConfig;
static ConfigData board1CpuShutdownOkConfig;

// map for storing list of gpio parameters whose config are to be read from x86
// power control json config
boost::container::flat_map<std::string, ConfigData*> powerSignalMap = {
    {"PowerOut", &powerOutConfig},
    {"PowerOk", &powerOkConfig},
    {"ResetOut", &resetOutConfig},
    {"NMIOut", &nmiOutConfig},
    {"SioPowerGood", &sioPwrGoodConfig},
    {"SioOnControl", &sioOnControlConfig},
    {"SIOS5", &sioS5Config},
    {"PostComplete", &postCompleteConfig},
    {"PowerButton", &powerButtonConfig},
    {"ResetButton", &resetButtonConfig},
    {"IdButton", &idButtonConfig},
    {"NMIButton", &nmiButtonConfig},
    {"SlotPower", &slotPowerConfig},
    {"NVL144PDBMainPowerEnable", &nvl144pdbMainPowerEnableConfig},
    {"NVL144PDBMainPowerOk", &nvl144pdbMainPowerOkConfig},
    {"C2PDBPSUPowerEnable", &c2pdbPSUPowerEnableConfig},
    {"C2PDBPSUPowerOk", &c2pdbPSUPowerOkConfig},
    {"C2PDB_12V_HPMEnable", &c2pdb_12V_HPMEnableConfig},
    {"C2PDB_12V_GPU1Enable", &c2pdb_12V_GPU1EnableConfig},
    {"C2PDB_12V_GPU2Enable", &c2pdb_12V_GPU2EnableConfig},
    {"C2PDB_12V_AICEnable", &c2pdb_12V_AICEnableConfig},
    {"USBPowerEnable", &usbPowerEnableConfig},
    {"Board0RunPowerPG", &board0RunPowerPGConfig},
    {"Board1RunPowerPG", &board1RunPowerPGConfig},
    {"Board0RunPowerEnable", &board0RunPowerEnableConfig},
    {"Board1RunPowerEnable", &board1RunPowerEnableConfig},
    {"Board0PreSystemReset", &board0PreSystemResetConfig},
    {"Board1PreSystemReset", &board1PreSystemResetConfig},
    {"CpuResetIndicator", &cpuResetIndicatorConfig},
    {"Board0CpuShutdownForce", &board0CpuShutdownForceConfig},
    {"Board1CpuShutdownForce", &board1CpuShutdownForceConfig},
    {"Board0CpuShutdownRequest", &board0CpuShutdownRequestConfig},
    {"Board1CpuShutdownRequest", &board1CpuShutdownRequestConfig},
    {"Board0CpuShutdownOk", &board0CpuShutdownOkConfig},
    {"Board1CpuShutdownOk", &board1CpuShutdownOkConfig}};

static std::string hostDbusName = "xyz.openbmc_project.State.Host";
static std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";
static std::string osDbusName = "xyz.openbmc_project.State.OperatingSystem";
static std::string buttonDbusName = "xyz.openbmc_project.Chassis.Buttons";
static std::string nmiDbusName = "xyz.openbmc_project.Control.Host.NMI";
static std::string rstCauseDbusName =
    "xyz.openbmc_project.Control.Host.RestartCause";
static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
#ifdef CHASSIS_SYSTEM_RESET
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSysIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSlotIface;
#endif
static std::shared_ptr<sdbusplus::asio::dbus_interface> powerButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> resetButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> osIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> idButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiOutIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> restartCauseIface;

static gpiod::line powerButtonMask;
static gpiod::line resetButtonMask;
static bool nmiButtonMasked = false;
#if IGNORE_SOFT_RESETS_DURING_POST
static bool ignoreNextSoftReset = false;
#endif

// This map contains all timer values that are to be read from json config
boost::container::flat_map<std::string, int> TimerMap = {
    {"PowerPulseMs", 200},
    {"ForceOffPulseMs", 15000},
    {"ResetPulseMs", 500},
    {"PowerCycleMs", 5000},
    {"SioPowerGoodWatchdogMs", 1000},
    {"PsPowerOKWatchdogMs", 8000},
    {"NVL144PdbMainPowerOkWatchdogMs", 10000},
    {"C2PdbPSUPowerOkWatchdogMs", 10000},
    {"HpmPowerGoodWatchdogMs", 15000},
    {"CpuResetWatchdogMs", 10000},
    {"CpuShutdownOkWatchdogMs", 10000},
    {"GracefulPowerOffS", (5 * 60)},
    {"WarmResetCheckMs", 500},
    {"PowerOffSaveMs", 7000},
    {"SlotPowerCycleMs", 200},
    {"DbusGetPropertyRetry", 1000}};

// Changed from default true to false
static bool nmiEnabled = false;
static bool nmiWhenPoweredOff = false;
static bool sioEnabled = false;

// Timers
// Time holding GPIOs asserted
static boost::asio::steady_timer gpioAssertTimer(io);
// Time between off and on during a power cycle
static boost::asio::steady_timer powerCycleTimer(io);
// Time OS gracefully powering off
static boost::asio::steady_timer gracefulPowerOffTimer(io);
// Time the warm reset check
static boost::asio::steady_timer warmResetCheckTimer(io);
// Time power supply power OK assertion on power-on
static boost::asio::steady_timer psPowerOKWatchdogTimer(io);
// Time PDB main power OK assertion/de-assertion in PDB Power sequencing
static boost::asio::steady_timer pdbMainPowerOkWatchdogTimer(io);
// Time HPM board power good assertion/de-assertion in HPM Power sequencing
static boost::asio::steady_timer hpmPowerGoodWatchdogTimer(io);
// Time CPU reset assertion on power-on
static boost::asio::steady_timer cpuResetWatchdogTimer(io);
// Time SIO power good assertion on power-on
static boost::asio::steady_timer sioPowerGoodWatchdogTimer(io);
// Time CPU shutdown OK assertion
static boost::asio::steady_timer cpuShutdownOkWatchdogTimer(io);
// Time power-off state save for power loss tracking
static boost::asio::steady_timer powerStateSaveTimer(io);
// POH timer
static boost::asio::steady_timer pohCounterTimer(io);
// Time when to allow restart cause updates
static boost::asio::steady_timer restartCauseTimer(io);
static boost::asio::steady_timer slotPowerCycleTimer(io);

// Map containing timers used for D-Bus get-property retries
static boost::container::flat_map<std::string, boost::asio::steady_timer>
    dBusRetryTimers;

// GPIO Lines and Event Descriptors
static gpiod::line psPowerOKLine;
static boost::asio::posix::stream_descriptor psPowerOKEvent(io);
static gpiod::line sioPowerGoodLine;
static boost::asio::posix::stream_descriptor sioPowerGoodEvent(io);
static gpiod::line sioOnControlLine;
static boost::asio::posix::stream_descriptor sioOnControlEvent(io);
static gpiod::line sioS5Line;
static boost::asio::posix::stream_descriptor sioS5Event(io);
static gpiod::line powerButtonLine;
static boost::asio::posix::stream_descriptor powerButtonEvent(io);
static gpiod::line resetButtonLine;
static boost::asio::posix::stream_descriptor resetButtonEvent(io);
static gpiod::line nmiButtonLine;
static boost::asio::posix::stream_descriptor nmiButtonEvent(io);
static gpiod::line idButtonLine;
static boost::asio::posix::stream_descriptor idButtonEvent(io);
static gpiod::line postCompleteLine;
static boost::asio::posix::stream_descriptor postCompleteEvent(io);
static gpiod::line nmiOutLine;
static gpiod::line slotPowerLine;

// New GPIO Lines for PDB and HPM Board control
// NVL144 PDB
static gpiod::line nvl144pdbMainPowerEnableLine;
static gpiod::line nvl144pdbMainPowerOkLine;
static boost::asio::posix::stream_descriptor nvl144pdbMainPowerOkEvent(io);
// -- end -- NVL144 PDB
// C2 PDB -- start--
static gpiod::line c2pdbPSUPowerEnableLine;
static gpiod::line c2pdbPSUPowerOkLine;
static boost::asio::posix::stream_descriptor c2pdbPSUPowerOkEvent(io);
static gpiod::line c2pdb_12V_HPMEnableLine;
static gpiod::line c2pdb_12V_GPU1EnableLine;
static gpiod::line c2pdb_12V_GPU2EnableLine;
static gpiod::line c2pdb_12V_AICEnableLine;
// -- end -- C2 PDB
static gpiod::line usbPowerEnableLine;
static gpiod::line board0RunPowerPGLine;
static boost::asio::posix::stream_descriptor board0RunPowerPGEvent(io);
static gpiod::line board1RunPowerPGLine;
static boost::asio::posix::stream_descriptor board1RunPowerPGEvent(io);
static gpiod::line board0RunPowerEnableLine;
static gpiod::line board1RunPowerEnableLine;
static gpiod::line board0PreSystemResetLine;
static gpiod::line board1PreSystemResetLine;
static gpiod::line cpuResetIndicatorLine;
static boost::asio::posix::stream_descriptor cpuResetIndicatorEvent(io);
static gpiod::line board0CpuShutdownForceLine;
static gpiod::line board1CpuShutdownForceLine;
static gpiod::line board0CpuShutdownRequestLine;
static gpiod::line board1CpuShutdownRequestLine;
static gpiod::line board0CpuShutdownOkLine;
static boost::asio::posix::stream_descriptor board0CpuShutdownOkEvent(io);
static gpiod::line board1CpuShutdownOkLine;
static boost::asio::posix::stream_descriptor board1CpuShutdownOkEvent(io);

static constexpr uint8_t beepPowerFail = 8;

static void beep(const uint8_t& beepPriority)
{
    lg2::info("Beep with priority: {BEEP_PRIORITY}", "BEEP_PRIORITY",
              beepPriority);

    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error(
                    "beep returned error with async_method_call (ec = {ERROR_MSG})",
                    "ERROR_MSG", ec.message());
                return;
            }
        },
        "xyz.openbmc_project.BeepCode", "/xyz/openbmc_project/BeepCode",
        "xyz.openbmc_project.BeepCode", "Beep", uint8_t(beepPriority));
}

enum class OperatingSystemStateStage
{
    Inactive,
    Standby,
};
static OperatingSystemStateStage operatingSystemState;
static constexpr std::string_view getOperatingSystemStateStage(
    const OperatingSystemStateStage stage)
{
    switch (stage)
    {
        case OperatingSystemStateStage::Inactive:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive";
            break;
        case OperatingSystemStateStage::Standby:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Standby";
            break;
        default:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive";
            break;
    }
};
static void setOperatingSystemState(const OperatingSystemStateStage stage)
{
    operatingSystemState = stage;
#if IGNORE_SOFT_RESETS_DURING_POST
    // If POST complete has asserted set ignoreNextSoftReset to false to avoid
    // masking soft resets after POST
    if (operatingSystemState == OperatingSystemStateStage::Standby)
    {
        ignoreNextSoftReset = false;
    }
#endif
    osIface->set_property("OperatingSystemState",
                          std::string(getOperatingSystemStateStage(stage)));

    lg2::info("Moving os state to {STATE} stage", "STATE",
              getOperatingSystemStateStage(stage));
}

enum class PowerState
{
    on,
    waitForPSPowerOK,
    waitForSIOPowerGood,
    off,
    transitionToOff,
    gracefulTransitionToOff,
    cycleOff,
    transitionToCycleOff,
    gracefulTransitionToCycleOff,
    checkForWarmReset,
    waitForPDBMainPowerOk,
    waitForPDBMainPowerOff,
    waitForHPMPowerGoodAssert,
    waitForHPMPowerGoodDeAssert,
    waitForCPUResetAssert,
    waitForCPUResetDeAssert,
    waitForCPUShutdownOk,
};
static PowerState powerState;
static std::string getPowerStateName(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPSPowerOK:
            return "Wait for Power Supply Power OK";
            break;
        case PowerState::waitForSIOPowerGood:
            return "Wait for SIO Power Good";
            break;
        case PowerState::off:
            return "Off";
            break;
        case PowerState::transitionToOff:
            return "Transition to Off";
            break;
        case PowerState::gracefulTransitionToOff:
            return "Graceful Transition to Off";
            break;
        case PowerState::cycleOff:
            return "Power Cycle Off";
            break;
        case PowerState::transitionToCycleOff:
            return "Transition to Power Cycle Off";
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return "Graceful Transition to Power Cycle Off";
            break;
        case PowerState::checkForWarmReset:
            return "Check for Warm Reset";
            break;
        case PowerState::waitForPDBMainPowerOk:
            return "Wait for PDB Main Power OK";
            break;
        case PowerState::waitForPDBMainPowerOff:
            return "Wait for PDB Main Power Off";
            break;
        case PowerState::waitForHPMPowerGoodAssert:
            return "Wait for HPM Power Good Assert";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            return "Wait for HPM Power Good De-Assert";
            break;
        case PowerState::waitForCPUResetAssert:
            return "Wait for CPU Reset Assert";
            break;
        case PowerState::waitForCPUResetDeAssert:
            return "Wait for CPU Reset De-Assert";
            break;
        case PowerState::waitForCPUShutdownOk:
            return "Wait for CPU Shutdown OK";
            break;
        default:
            return "unknown state: " + std::to_string(static_cast<int>(state));
            break;
    }
}
static void logStateTransition(const PowerState state)
{
    lg2::info("Host{HOST}: Moving to \"{STATE}\" state", "HOST", node, "STATE",
              getPowerStateName(state));
}

enum class Event
{
    psPowerOKAssert,
    psPowerOKDeAssert,
    sioPowerGoodAssert,
    sioPowerGoodDeAssert,
    sioS5Assert,
    sioS5DeAssert,
    pltRstAssert,
    pltRstDeAssert,
    postCompleteAssert,
    postCompleteDeAssert,
    powerButtonPressed,
    resetButtonPressed,
    powerCycleTimerExpired,
    psPowerOKWatchdogTimerExpired,
    pdbMainPowerOkWatchdogTimerExpired,
    hpmPowerGoodWatchdogTimerExpired,
    cpuResetWatchdogTimerExpired,
    cpuShutdownOkWatchdogTimerExpired,
    sioPowerGoodWatchdogTimerExpired,
    gracefulPowerOffTimerExpired,
    powerOnRequest,
    powerOffRequest,
    powerCycleRequest,
    resetRequest,
    gracefulPowerOffRequest,
    gracefulPowerCycleRequest,
    warmResetDetected,
    nvl144pdbMainPowerOkAssert,
    nvl144pdbMainPowerOkDeAssert,
    c2pdbPSUPowerOkAssert,
    c2pdbPSUPowerOkDeAssert,
    board0RunPowerPGAssert,
    board0RunPowerPGDeAssert,
    board1RunPowerPGAssert,
    board1RunPowerPGDeAssert,
    cpuResetIndicatorAssert,
    cpuResetIndicatorDeAssert,
    board0CpuShutdownOkAssert,
    board0CpuShutdownOkDeAssert,
    board1CpuShutdownOkAssert,
    board1CpuShutdownOkDeAssert,
};
static std::string getEventName(Event event)
{
    switch (event)
    {
        case Event::psPowerOKAssert:
            return "power supply power OK assert";
            break;
        case Event::psPowerOKDeAssert:
            return "power supply power OK de-assert";
            break;
        case Event::sioPowerGoodAssert:
            return "SIO power good assert";
            break;
        case Event::sioPowerGoodDeAssert:
            return "SIO power good de-assert";
            break;
        case Event::sioS5Assert:
            return "SIO S5 assert";
            break;
        case Event::sioS5DeAssert:
            return "SIO S5 de-assert";
            break;
        case Event::pltRstAssert:
            return "PLT_RST assert";
            break;
        case Event::pltRstDeAssert:
            return "PLT_RST de-assert";
            break;
        case Event::postCompleteAssert:
            return "POST Complete assert";
            break;
        case Event::postCompleteDeAssert:
            return "POST Complete de-assert";
            break;
        case Event::powerButtonPressed:
            return "power button pressed";
            break;
        case Event::resetButtonPressed:
            return "reset button pressed";
            break;
        case Event::powerCycleTimerExpired:
            return "power cycle timer expired";
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
            break;
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            return "PDB main power OK watchdog timer expired";
            break;
        case Event::hpmPowerGoodWatchdogTimerExpired:
            return "HPM power good watchdog timer expired";
            break;
        case Event::cpuResetWatchdogTimerExpired:
            return "CPU reset watchdog timer expired";
            break;
        case Event::cpuShutdownOkWatchdogTimerExpired:
            return "CPU shutdown OK watchdog timer expired";
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            return "SIO power good watchdog timer expired";
            break;
        case Event::gracefulPowerOffTimerExpired:
            return "graceful power-off timer expired";
            break;
        case Event::powerOnRequest:
            return "power-on request";
            break;
        case Event::powerOffRequest:
            return "power-off request";
            break;
        case Event::powerCycleRequest:
            return "power-cycle request";
            break;
        case Event::resetRequest:
            return "reset request";
            break;
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
            break;
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
            break;
        case Event::warmResetDetected:
            return "warm reset detected";
            break;
        case Event::nvl144pdbMainPowerOkAssert:
            return "NVL144 PDB main power OK assert";
            break;
        case Event::nvl144pdbMainPowerOkDeAssert:
            return "NVL144 PDB main power OK de-assert";
            break;
        case Event::c2pdbPSUPowerOkAssert:
            return "C2 PDB main power OK assert";
            break;
        case Event::c2pdbPSUPowerOkDeAssert:
            return "C2 PDB main power OK de-assert";
            break;
        case Event::board0RunPowerPGAssert:
            return "Board 0 run power PG assert";
            break;
        case Event::board0RunPowerPGDeAssert:
            return "Board 0 run power PG de-assert";
            break;
        case Event::board1RunPowerPGAssert:
            return "Board 1 run power PG assert";
            break;
        case Event::board1RunPowerPGDeAssert:
            return "Board 1 run power PG de-assert";
            break;
        case Event::cpuResetIndicatorAssert:
            return "CPU reset indicator assert";
            break;
        case Event::cpuResetIndicatorDeAssert:
            return "CPU reset indicator de-assert";
            break;
        case Event::board0CpuShutdownOkAssert:
            return "Board 0 CPU shutdown OK assert";
            break;
        case Event::board0CpuShutdownOkDeAssert:
            return "Board 0 CPU shutdown OK de-assert";
            break;
        case Event::board1CpuShutdownOkAssert:
            return "Board 1 CPU shutdown OK assert";
            break;
        case Event::board1CpuShutdownOkDeAssert:
            return "Board 1 CPU shutdown OK de-assert";
            break;
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
            break;
    }
}
static void logEvent(const std::string_view stateHandler, const Event event)
{
    lg2::info("{STATE_HANDLER}: {EVENT} event received", "STATE_HANDLER",
              stateHandler, "EVENT", getEventName(event));
}

// Power state handlers
static void powerStateOn(const Event event);
static void powerStateWaitForPSPowerOK(const Event event);
static void powerStateWaitForSIOPowerGood(const Event event);
static void powerStateOff(const Event event);
static void powerStateTransitionToOff(const Event event);
static void powerStateGracefulTransitionToOff(const Event event);
static void powerStateCycleOff(const Event event);
static void powerStateTransitionToCycleOff(const Event event);
static void powerStateGracefulTransitionToCycleOff(const Event event);
static void powerStateCheckForWarmReset(const Event event);
static void powerStateWaitForPDBMainPowerOk(const Event event);
static void powerStateWaitForPDBMainPowerOff(const Event event);
static void powerStateWaitForHPMPowerGoodAssert(const Event event);
static void powerStateWaitForHPMPowerGoodDeAssert(const Event event);
static void powerStateWaitForCPUResetAssert(const Event event);
static void powerStateWaitForCPUResetDeAssert(const Event event);
static void powerStateWaitForCPUShutdownOk(const Event event);

// Function forward declarations
static int loadConfigValues();
static void detectBoardPresence();
static int getProperty(const ConfigData& configData);
static void nmiSourcePropertyMonitor(void);
static void nmiReset(void);
static sdbusplus::bus::match_t dbusGPIOMatcher(const ConfigData& cfg, std::function<void(bool)>& onMatch);

// GPIO handler forward declarations
static void psPowerOKHandler(bool state);
static void sioPowerGoodHandler(bool state);
static void sioOnControlHandler(bool state);
static void sioS5Handler(bool state);
static void powerButtonHandler(bool state);
static void resetButtonHandler(bool state);
static void nmiButtonHandler(bool state);
static void idButtonHandler(bool state);
static void postCompleteHandler(bool state);
static void nvl144pdbMainPowerOkHandler(bool state);
static void c2pdbPSUPowerOkHandler(bool state);
static void board0RunPowerPGHandler(bool state);
static void board1RunPowerPGHandler(bool state);
static void cpuResetIndicatorHandler(bool state);
static void board0CpuShutdownOkHandler(bool state);
static void board1CpuShutdownOkHandler(bool state);

static std::function<void(const Event)> getPowerStateHandler(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return powerStateOn;
            break;
        case PowerState::waitForPSPowerOK:
            return powerStateWaitForPSPowerOK;
            break;
        case PowerState::waitForSIOPowerGood:
            return powerStateWaitForSIOPowerGood;
            break;
        case PowerState::off:
            return powerStateOff;
            break;
        case PowerState::transitionToOff:
            return powerStateTransitionToOff;
            break;
        case PowerState::gracefulTransitionToOff:
            return powerStateGracefulTransitionToOff;
            break;
        case PowerState::cycleOff:
            return powerStateCycleOff;
            break;
        case PowerState::transitionToCycleOff:
            return powerStateTransitionToCycleOff;
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return powerStateGracefulTransitionToCycleOff;
            break;
        case PowerState::checkForWarmReset:
            return powerStateCheckForWarmReset;
            break;
        case PowerState::waitForPDBMainPowerOk:
            return powerStateWaitForPDBMainPowerOk;
            break;
        case PowerState::waitForPDBMainPowerOff:
            return powerStateWaitForPDBMainPowerOff;
            break;
        case PowerState::waitForHPMPowerGoodAssert:
            return powerStateWaitForHPMPowerGoodAssert;
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            return powerStateWaitForHPMPowerGoodDeAssert;
            break;
        case PowerState::waitForCPUResetAssert:
            return powerStateWaitForCPUResetAssert;
            break;
        case PowerState::waitForCPUResetDeAssert:
            return powerStateWaitForCPUResetDeAssert;
            break;
        case PowerState::waitForCPUShutdownOk:
            return powerStateWaitForCPUShutdownOk;
            break;
        default:
            return nullptr;
            break;
    }
};

static void sendPowerControlEvent(const Event event)
{
    std::function<void(const Event)> handler = getPowerStateHandler(powerState);
    if (handler == nullptr)
    {
        lg2::error("Failed to find handler for power state: {STATE}", "STATE",
                   static_cast<int>(powerState));
        return;
    }
    handler(event);
}

static uint64_t getCurrentTimeMs()
{
    struct timespec time = {};

    if (clock_gettime(CLOCK_REALTIME, &time) < 0)
    {
        return 0;
    }
    uint64_t currentTimeMs = static_cast<uint64_t>(time.tv_sec) * 1000;
    currentTimeMs += static_cast<uint64_t>(time.tv_nsec) / 1000 / 1000;

    return currentTimeMs;
}

// Use Action Struct to report host state (context matters)
static constexpr std::string_view getHostState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::gracefulTransitionToOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running"; // (upstream)
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        case PowerState::transitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::cycleOff:
        case PowerState::checkForWarmReset:
        case PowerState::waitForPDBMainPowerOk:
            // Only called druing PowerContext::action == PowerAction::POWER_ON
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            break;
        case PowerState::waitForPDBMainPowerOff:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            }
            else if(powerContext.action == PowerAction::FORCE_OFF 
                    || powerContext.action == PowerAction::GRACE_OFF
                    || powerContext.action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            }
            break;
        case PowerState::waitForHPMPowerGoodAssert:
            // Only called druing PowerContext::action == PowerAction::POWER_ON
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            break;
        case PowerState::waitForHPMPowerGoodDeAssert:
            // Only called druing shutdown actions
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            break;
        case PowerState::waitForCPUResetAssert:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            }
            else if(powerContext.action == PowerAction::FORCE_OFF 
                    || powerContext.action == PowerAction::GRACE_OFF
                    || powerContext.action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            }
            break;
        case PowerState::waitForCPUResetDeAssert:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToRunning";
            }
            else if(powerContext.action == PowerAction::FORCE_OFF 
                    || powerContext.action == PowerAction::GRACE_OFF
                    || powerContext.action == PowerAction::HOST_INITIATED_SHUTDOWN)
            {
                return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            }
            break;
        case PowerState::waitForCPUShutdownOk:
            return "xyz.openbmc_project.State.Host.HostState.TransitioningToOff";
            break;
        default:
            return "";
            break;
    }
};
static constexpr std::string_view getChassisState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::cycleOff:
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
        case PowerState::waitForHPMPowerGoodAssert:
        case PowerState::waitForHPMPowerGoodDeAssert:
        case PowerState::waitForCPUResetAssert:
        case PowerState::waitForCPUResetDeAssert:
        case PowerState::waitForCPUShutdownOk:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
#ifdef CHASSIS_SYSTEM_RESET
enum class SlotPowerState
{
    on,
    off,
};
static SlotPowerState slotPowerState;
static constexpr std::string_view getSlotState(const SlotPowerState state)
{
    switch (state)
    {
        case SlotPowerState::on:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case SlotPowerState::off:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
static void setSlotPowerState(const SlotPowerState state)
{
    slotPowerState = state;
    chassisSlotIface->set_property("CurrentPowerState",
                                   std::string(getSlotState(slotPowerState)));
    chassisSlotIface->set_property("LastStateChangeTime", getCurrentTimeMs());
}
#endif
static void savePowerState(const PowerState state)
{
    powerStateSaveTimer.expires_after(
        std::chrono::milliseconds(TimerMap["PowerOffSaveMs"]));
    powerStateSaveTimer.async_wait([state](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Power-state save async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            return;
        }
        appState.set(PersistentState::Params::PowerState,
                     std::string{getChassisState(state)});
    });
}
static void setPowerState(const PowerState state)
{
    powerState = state;
    logStateTransition(state);

    hostIface->set_property("CurrentHostState",
                            std::string(getHostState(powerState)));

    chassisIface->set_property("CurrentPowerState",
                               std::string(getChassisState(powerState)));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());

    // Save the power state for the restore policy
    savePowerState(state);
}

enum class RestartCause
{
    command,
    resetButton,
    powerButton,
    watchdog,
    powerPolicyOn,
    powerPolicyRestore,
    softReset,
};
static boost::container::flat_set<RestartCause> causeSet;
static std::string getRestartCause(RestartCause cause)
{
    switch (cause)
    {
        case RestartCause::command:
            return "xyz.openbmc_project.State.Host.RestartCause.IpmiCommand";
            break;
        case RestartCause::resetButton:
            return "xyz.openbmc_project.State.Host.RestartCause.ResetButton";
            break;
        case RestartCause::powerButton:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerButton";
            break;
        case RestartCause::watchdog:
            return "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer";
            break;
        case RestartCause::powerPolicyOn:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerPolicyAlwaysOn";
            break;
        case RestartCause::powerPolicyRestore:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerPolicyPreviousState";
            break;
        case RestartCause::softReset:
            return "xyz.openbmc_project.State.Host.RestartCause.SoftReset";
            break;
        default:
            return "xyz.openbmc_project.State.Host.RestartCause.Unknown";
            break;
    }
}
static void addRestartCause(const RestartCause cause)
{
    // Add this to the set of causes for this restart
    causeSet.insert(cause);
}
static void clearRestartCause()
{
    // Clear the set for the next restart
    causeSet.clear();
}
static void setRestartCauseProperty(const std::string& cause)
{
    lg2::info("RestartCause set to {RESTART_CAUSE}", "RESTART_CAUSE", cause);
    restartCauseIface->set_property("RestartCause", cause);
}

#ifdef USE_ACBOOT
static void resetACBootProperty()
{
    if ((causeSet.contains(RestartCause::command)) ||
        (causeSet.contains(RestartCause::softReset)))
    {
        conn->async_method_call(
            [](boost::system::error_code ec) {
                if (ec)
                {
                    lg2::error("failed to reset ACBoot property");
                }
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/control/host0/ac_boot",
            "org.freedesktop.DBus.Properties", "Set",
            "xyz.openbmc_project.Common.ACBoot", "ACBoot",
            std::variant<std::string>{"False"});
    }
}
#endif // USE_ACBOOT

static void setRestartCause()
{
    // Determine the actual restart cause based on the set of causes
    std::string restartCause =
        "xyz.openbmc_project.State.Host.RestartCause.Unknown";
    if (causeSet.contains(RestartCause::watchdog))
    {
        restartCause = getRestartCause(RestartCause::watchdog);
    }
    else if (causeSet.contains(RestartCause::command))
    {
        restartCause = getRestartCause(RestartCause::command);
    }
    else if (causeSet.contains(RestartCause::resetButton))
    {
        restartCause = getRestartCause(RestartCause::resetButton);
    }
    else if (causeSet.contains(RestartCause::powerButton))
    {
        restartCause = getRestartCause(RestartCause::powerButton);
    }
    else if (causeSet.contains(RestartCause::powerPolicyOn))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyOn);
    }
    else if (causeSet.contains(RestartCause::powerPolicyRestore))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyRestore);
    }
    else if (causeSet.contains(RestartCause::softReset))
    {
#if IGNORE_SOFT_RESETS_DURING_POST
        if (ignoreNextSoftReset)
        {
            ignoreNextSoftReset = false;
            return;
        }
#endif
        restartCause = getRestartCause(RestartCause::softReset);
    }

    setRestartCauseProperty(restartCause);
}

static void systemPowerGoodFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: system power good failed to assert (VR failure)",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.SystemPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        TimerMap["SioPowerGoodWatchdogMs"], NULL);
}

static void psPowerOKFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: power supply power good failed to assert",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.PowerSupplyPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        TimerMap["PsPowerOKWatchdogMs"], NULL);
}

static void powerRestorePolicyLog()
{
    sd_journal_send("MESSAGE=PowerControl: power restore policy applied",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerRestorePolicyApplied", NULL);
}

static void powerButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: power button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerButtonPressed", NULL);
}

static void resetButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: reset button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.ResetButtonPressed", NULL);
}

static void nmiButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIButtonPressed", NULL);
}

static void nmiDiagIntLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI Diagnostic Interrupt",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIDiagnosticInterrupt", NULL);
}

PersistentState::PersistentState()
{
    // create the power control directory if it doesn't exist
    std::error_code ec;
    if (!(std::filesystem::create_directories(powerControlDir, ec)))
    {
        if (ec.value() != 0)
        {
            lg2::error("failed to create {DIR_NAME}: {ERROR_MSG}", "DIR_NAME",
                       powerControlDir.string(), "ERROR_MSG", ec.message());
            throw std::runtime_error("Failed to create state directory");
        }
    }

    // read saved state, it's ok, if the file doesn't exists
    std::ifstream appStateStream(powerControlDir / stateFile);
    if (!appStateStream.is_open())
    {
        lg2::info("Cannot open state file \'{PATH}\'", "PATH",
                  std::string(powerControlDir / stateFile));
        stateData = nlohmann::json({});
        return;
    }
    try
    {
        appStateStream >> stateData;
        if (stateData.is_discarded())
        {
            lg2::info("Cannot parse state file \'{PATH}\'", "PATH",
                      std::string(powerControlDir / stateFile));
            stateData = nlohmann::json({});
            return;
        }
    }
    catch (const std::exception& ex)
    {
        lg2::info("Cannot read state file \'{PATH}\'", "PATH",
                  std::string(powerControlDir / stateFile));
        stateData = nlohmann::json({});
        return;
    }
}
PersistentState::~PersistentState()
{
    saveState();
}
const std::string PersistentState::get(Params parameter)
{
    auto val = stateData.find(getName(parameter));
    if (val != stateData.end())
    {
        return val->get<std::string>();
    }
    return getDefault(parameter);
}
void PersistentState::set(Params parameter, const std::string& value)
{
    stateData[getName(parameter)] = value;
    saveState();
}

const std::string PersistentState::getName(const Params parameter)
{
    switch (parameter)
    {
        case Params::PowerState:
            return "PowerState";
    }
    return "";
}
const std::string PersistentState::getDefault(const Params parameter)
{
    switch (parameter)
    {
        case Params::PowerState:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
    }
    return "";
}
void PersistentState::saveState()
{
    std::ofstream appStateStream(powerControlDir / stateFile, std::ios::trunc);
    if (!appStateStream.is_open())
    {
        lg2::error("Cannot write state file \'{PATH}\'", "PATH",
                   std::string(powerControlDir / stateFile));
        return;
    }
    appStateStream << stateData.dump(indentationSize);
}

static constexpr const char* setingsService = "xyz.openbmc_project.Settings";
static constexpr const char* powerRestorePolicyIface =
    "xyz.openbmc_project.Control.Power.RestorePolicy";
#ifdef USE_ACBOOT
static constexpr const char* powerACBootObject =
    "/xyz/openbmc_project/control/host0/ac_boot";
static constexpr const char* powerACBootIface =
    "xyz.openbmc_project.Common.ACBoot";
#endif // USE_ACBOOT

namespace match_rules = sdbusplus::bus::match::rules;

static int powerRestoreConfigHandler(sd_bus_message* m, void* context,
                                     sd_bus_error*)
{
    if (context == nullptr || m == nullptr)
    {
        throw std::runtime_error("Invalid match");
    }
    sdbusplus::message_t message(m);
    PowerRestoreController* powerRestore =
        static_cast<PowerRestoreController*>(context);

    if (std::string(message.get_member()) == "InterfacesAdded")
    {
        sdbusplus::message::object_path path;
        boost::container::flat_map<std::string, dbusPropertiesList> data;

        message.read(path, data);

        for (auto& [iface, properties] : data)
        {
            if ((iface == powerRestorePolicyIface)
#ifdef USE_ACBOOT
                || (iface == powerACBootIface)
#endif // USE_ACBOOT
            )
            {
                powerRestore->setProperties(properties);
            }
        }
    }
    else if (std::string(message.get_member()) == "PropertiesChanged")
    {
        std::string interfaceName;
        dbusPropertiesList propertiesChanged;

        message.read(interfaceName, propertiesChanged);

        powerRestore->setProperties(propertiesChanged);
    }
    return 1;
}

void PowerRestoreController::run()
{
    std::string powerRestorePolicyObject =
        "/xyz/openbmc_project/control/host" + node + "/power_restore_policy";
    powerRestorePolicyLog();
    // this list only needs to be created once
    if (matches.empty())
    {
        matches.emplace_back(
            *conn,
            match_rules::interfacesAdded() +
                match_rules::argNpath(0, powerRestorePolicyObject) +
                match_rules::sender(setingsService),
            powerRestoreConfigHandler, this);
#ifdef USE_ACBOOT
        matches.emplace_back(*conn,
                             match_rules::interfacesAdded() +
                                 match_rules::argNpath(0, powerACBootObject) +
                                 match_rules::sender(setingsService),
                             powerRestoreConfigHandler, this);
        matches.emplace_back(*conn,
                             match_rules::propertiesChanged(powerACBootObject,
                                                            powerACBootIface) +
                                 match_rules::sender(setingsService),
                             powerRestoreConfigHandler, this);
#endif // USE_ACBOOT
    }

    // Check if it's already on DBus
    conn->async_method_call(
        [this](boost::system::error_code ec,
               const dbusPropertiesList properties) {
            if (ec)
            {
                return;
            }
            setProperties(properties);
        },
        setingsService, powerRestorePolicyObject,
        "org.freedesktop.DBus.Properties", "GetAll", powerRestorePolicyIface);

#ifdef USE_ACBOOT
    // Check if it's already on DBus
    conn->async_method_call(
        [this](boost::system::error_code ec,
               const dbusPropertiesList properties) {
            if (ec)
            {
                return;
            }
            setProperties(properties);
        },
        setingsService, powerACBootObject, "org.freedesktop.DBus.Properties",
        "GetAll", powerACBootIface);
#endif
}

void PowerRestoreController::setProperties(const dbusPropertiesList& props)
{
    for (auto& [property, propValue] : props)
    {
        if (property == "PowerRestorePolicy")
        {
            const std::string* value = std::get_if<std::string>(&propValue);
            if (value == nullptr)
            {
                lg2::error("Unable to read Power Restore Policy");
                continue;
            }
            powerRestorePolicy = *value;
        }
        else if (property == "PowerRestoreDelay")
        {
            const uint64_t* value = std::get_if<uint64_t>(&propValue);
            if (value == nullptr)
            {
                lg2::error("Unable to read Power Restore Delay");
                continue;
            }
            powerRestoreDelay = *value / 1000000; // usec to sec
        }
#ifdef USE_ACBOOT
        else if (property == "ACBoot")
        {
            const std::string* value = std::get_if<std::string>(&propValue);
            if (value == nullptr)
            {
                lg2::error("Unable to read AC Boot status");
                continue;
            }
            acBoot = *value;
        }
#endif // USE_ACBOOT
    }
    invokeIfReady();
}

void PowerRestoreController::invokeIfReady()
{
    if ((powerRestorePolicy.empty()) || (powerRestoreDelay < 0))
    {
        return;
    }
#ifdef USE_ACBOOT
    if (acBoot.empty() || acBoot == "Unknown")
    {
        return;
    }
#endif

    matches.clear();
    if (!timerFired)
    {
        // Calculate the delay from now to meet the requested delay
        // Subtract the approximate uboot time
        static constexpr const int ubootSeconds = 20;
        int delay = powerRestoreDelay - ubootSeconds;
        // Subtract the time since boot
        struct sysinfo info = {};
        if (sysinfo(&info) == 0)
        {
            delay -= info.uptime;
        }

        if (delay > 0)
        {
            powerRestoreTimer.expires_after(std::chrono::seconds(delay));
            lg2::info("Power Restore delay of {DELAY} seconds started", "DELAY",
                      delay);
            powerRestoreTimer.async_wait([this](const boost::system::error_code
                                                    ec) {
                if (ec)
                {
                    // operation_aborted is expected if timer is canceled before
                    // completion.
                    if (ec == boost::asio::error::operation_aborted)
                    {
                        return;
                    }
                    lg2::error(
                        "power restore policy async_wait failed: {ERROR_MSG}",
                        "ERROR_MSG", ec.message());
                }
                else
                {
                    lg2::info("Power Restore delay timer expired");
                }
                invoke();
            });
            timerFired = true;
        }
        else
        {
            invoke();
        }
    }
}

void PowerRestoreController::invoke()
{
    // we want to run Power Restore only once
    if (policyInvoked)
    {
        return;
    }
    policyInvoked = true;

    lg2::info("Invoking Power Restore Policy {POLICY}", "POLICY",
              powerRestorePolicy);
    if (powerRestorePolicy ==
        "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.AlwaysOn")
    {
        sendPowerControlEvent(Event::powerOnRequest);
        setRestartCauseProperty(getRestartCause(RestartCause::powerPolicyOn));
    }
    else if (powerRestorePolicy ==
             "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.Restore")
    {
        if (wasPowerDropped())
        {
            lg2::info("Power was dropped, restoring Host On state");
            sendPowerControlEvent(Event::powerOnRequest);
            setRestartCauseProperty(
                getRestartCause(RestartCause::powerPolicyRestore));
        }
        else
        {
            lg2::info("No power drop, restoring Host Off state");
        }
    }
    // We're done with the previous power state for the restore policy, so store
    // the current state
    savePowerState(powerState);
}

bool PowerRestoreController::wasPowerDropped()
{
    std::string state = appState.get(PersistentState::Params::PowerState);
    return state == "xyz.openbmc_project.State.Chassis.PowerState.On";
}

static void waitForGPIOEvent(
    const std::string& name, const std::function<void(bool)>& eventHandler,
    gpiod::line& line, boost::asio::posix::stream_descriptor& event)
{
    event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, eventHandler, &line,
         &event](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("{GPIO_NAME} fd handler error: {ERROR_MSG}",
                           "GPIO_NAME", name, "ERROR_MSG", ec.message());
                // TODO: throw here to force power-control to
                // restart?
                return;
            }
            gpiod::line_event line_event = line.event_read();
            eventHandler(line_event.event_type ==
                         gpiod::line_event::RISING_EDGE);
            waitForGPIOEvent(name, eventHandler, line, event);
        });
}

static bool requestGPIOEvents(
    const std::string& name, const std::function<void(bool)>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", name);
        return false;
    }

    try
    {
        gpioLine.request({appName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to request events for {GPIO_NAME}: {ERROR}",
                   "GPIO_NAME", name, "ERROR", e);
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        lg2::error("Failed to get {GPIO_NAME} fd", "GPIO_NAME", name);
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    waitForGPIOEvent(name, handler, gpioLine, gpioEventDescriptor);
    return true;
}


static bool setGPIOOutput(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    if(!gpioLine)
    {
        gpioLine = gpiod::find_line(name);
        if(!gpioLine)
        {
            lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", name);
            return false;
        }
    }
    // Request GPIO output to specified value
    if(!gpioLine.is_requested())
    {
        try
        {
            gpioLine.request({appName, gpiod::line_request::DIRECTION_OUTPUT, {}},
                            value);
        }
        catch (const std::exception& e)
        {
            lg2::error("Failed to request {GPIO_NAME} output: {ERROR}", "GPIO_NAME",
                    name, "ERROR", e);
            return false;
        }
    }
    else
    {
        try 
        {
            gpioLine.set_value(value);
        }
        catch (const std::exception& e)
        {
            lg2::error("Failed to set {GPIO_NAME} value: {ERROR}",
                       "GPIO_NAME", name, "ERROR", e);
            return false;
        }
    }

    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", name,
              "GPIO_VALUE", value);
    return true;
    
}



static int setMaskedGPIOOutputForMs(gpiod::line& maskedGPIOLine,
                                    const std::string& name, const int value,
                                    const int durationMs)
{
    // Set the masked GPIO line to the specified value
    maskedGPIOLine.set_value(value);
    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", name,
              "GPIO_VALUE", value);
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [maskedGPIOLine, value, name](const boost::system::error_code ec) {
            // Set the masked GPIO line back to the opposite value
            maskedGPIOLine.set_value(!value);
            lg2::info("{GPIO_NAME} released", "GPIO_NAME", name);
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                               "GPIO_NAME", name, "ERROR_MSG", ec.message());
                }
            }
        });
    return 0;
}

static int setGPIOOutputForMs(const ConfigData& config, const int value,
                              const int durationMs)
{
    // If the requested GPIO is masked, use the mask line to set the output
    if (powerButtonMask && config.lineName == powerOutConfig.lineName)
    {
        return setMaskedGPIOOutputForMs(powerButtonMask, config.lineName, value,
                                        durationMs);
    }
    if (resetButtonMask && config.lineName == resetOutConfig.lineName)
    {
        return setMaskedGPIOOutputForMs(resetButtonMask, config.lineName, value,
                                        durationMs);
    }

    // No mask set, so request and set the GPIO normally
    gpiod::line gpioLine;
    if (!setGPIOOutput(config.lineName, value, gpioLine))
    {
        return -1;
    }
    const std::string name = config.lineName;

    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [gpioLine, value, name](const boost::system::error_code ec) {
            // Set the GPIO line back to the opposite value
            gpioLine.set_value(!value);
            lg2::info("{GPIO_NAME} released", "GPIO_NAME", name);
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                               "GPIO_NAME", name, "ERROR_MSG", ec.message());
                }
            }
        });
    return 0;
}

static int assertGPIOForMs(const ConfigData& config, const int durationMs)
{
    return setGPIOOutputForMs(config, config.polarity, durationMs);
}

static void powerOn()
{
    assertGPIOForMs(powerOutConfig, TimerMap["PowerPulseMs"]);
}
#ifdef CHASSIS_SYSTEM_RESET
static int slotPowerOn()
{
    if (power_control::slotPowerState != power_control::SlotPowerState::on)
    {
        slotPowerLine.set_value(1);

        if (slotPowerLine.get_value() > 0)
        {
            setSlotPowerState(SlotPowerState::on);
            lg2::info("Slot Power is switched On\n");
        }
        else
        {
            return -1;
        }
    }
    else
    {
        lg2::info("Slot Power is already in 'On' state\n");
        return -1;
    }
    return 0;
}
static int slotPowerOff()
{
    if (power_control::slotPowerState != power_control::SlotPowerState::off)
    {
        slotPowerLine.set_value(0);

        if (!(slotPowerLine.get_value() > 0))
        {
            setSlotPowerState(SlotPowerState::off);
            setPowerState(PowerState::off);
            lg2::info("Slot Power is switched Off\n");
        }
        else
        {
            return -1;
        }
    }
    else
    {
        lg2::info("Slot Power is already in 'Off' state\n");
        return -1;
    }
    return 0;
}
static void slotPowerCycle()
{
    lg2::info("Slot Power Cycle started\n");
    slotPowerOff();
    slotPowerCycleTimer.expires_after(
        std::chrono::milliseconds(TimerMap["SlotPowerCycleMs"]));
    slotPowerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "Slot Power cycle timer async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("Slot Power cycle timer canceled\n");
            return;
        }
        lg2::info("Slot Power cycle timer completed\n");
        slotPowerOn();
        lg2::info("Slot Power Cycle Completed\n");
    });
}
#endif
static void gracefulPowerOff()
{
    assertGPIOForMs(powerOutConfig, TimerMap["PowerPulseMs"]);
}

static void forcePowerOff()
{
    if (assertGPIOForMs(powerOutConfig, TimerMap["ForceOffPulseMs"]) < 0)
    {
        return;
    }

    // If the force off timer expires, then the power-button override failed
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Force power off async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            return;
        }

        lg2::error("Power-button override failed. Not sure what to do now.");
    });
}

static void reset()
{
    assertGPIOForMs(resetOutConfig, TimerMap["ResetPulseMs"]);
}

static void gracefulPowerOffTimerStart()
{
    lg2::info("Graceful power-off timer started");
    gracefulPowerOffTimer.expires_after(
        std::chrono::seconds(TimerMap["GracefulPowerOffS"]));
    gracefulPowerOffTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Graceful power-off async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("Graceful power-off timer canceled");
            return;
        }
        lg2::info("Graceful power-off timer completed");
        sendPowerControlEvent(Event::gracefulPowerOffTimerExpired);
    });
}

static void powerCycleTimerStart()
{
    lg2::info("Power-cycle timer started");
    powerCycleTimer.expires_after(
        std::chrono::milliseconds(TimerMap["PowerCycleMs"]));
    powerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Power-cycle async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("Power-cycle timer canceled");
            return;
        }
        lg2::info("Power-cycle timer completed");
        sendPowerControlEvent(Event::powerCycleTimerExpired);
    });
}

static void psPowerOKWatchdogTimerStart()
{
    lg2::info("power supply power OK watchdog timer started");
    psPowerOKWatchdogTimer.expires_after(
        std::chrono::milliseconds(TimerMap["PsPowerOKWatchdogMs"]));
    psPowerOKWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "power supply power OK watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("power supply power OK watchdog timer canceled");
            return;
        }
        lg2::info("power supply power OK watchdog timer expired");
        sendPowerControlEvent(Event::psPowerOKWatchdogTimerExpired);
    });
}

static void pdbMainPowerOkWatchdogTimerStart(int timeoutMs = -1)
{
    // Use provided timeout or default from TimerMap
    int timeout = 0;
    if (timeoutMs > 0)
    {
        timeout = timeoutMs;
    }
    else if(powerContext.presence.nvl144_pdb)
    {
        timeout = TimerMap["NVL144PdbMainPowerOkWatchdogMs"];
    }
    else if(powerContext.presence.c2_pdb)
    {
        timeout = TimerMap["C2PdbPSUPowerOkWatchdogMs"];
    }
    else
    {
        lg2::error("No PDB present");
        return;
    }
    
    lg2::info("PDB main power OK watchdog timer started with {TIMEOUT_MS}ms timeout",
              "TIMEOUT_MS", timeout);
    pdbMainPowerOkWatchdogTimer.expires_after(
        std::chrono::milliseconds(timeout));
    pdbMainPowerOkWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "PDB main power OK watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("PDB main power OK watchdog timer canceled");
            return;
        }
        lg2::info("PDB main power OK watchdog timer expired");
        sendPowerControlEvent(Event::pdbMainPowerOkWatchdogTimerExpired);
    });
}

static void cpuResetWatchdogTimerStart(int timeoutMs = -1)
{
    // Use provided timeout or default from TimerMap
    int timeout = (timeoutMs == -1) ? TimerMap["CpuResetWatchdogMs"] : timeoutMs;
    
    lg2::info("CPU reset watchdog timer started with {TIMEOUT_MS}ms timeout",
              "TIMEOUT_MS", timeout);
    cpuResetWatchdogTimer.expires_after(
        std::chrono::milliseconds(timeout));
    cpuResetWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "CPU reset watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("CPU reset watchdog timer canceled");
            return;
        }
        lg2::info("CPU reset watchdog timer expired");
        sendPowerControlEvent(Event::cpuResetWatchdogTimerExpired);
    });
}

static void cpuShutdownOkWatchdogTimerStart(int timeoutMs = -1)
{
    // Use provided timeout or default from TimerMap
    int timeout = (timeoutMs == -1) ? TimerMap["CpuShutdownOkWatchdogMs"] : timeoutMs;
    
    lg2::info("CPU shutdown OK watchdog timer started with {TIMEOUT_MS}ms timeout",
              "TIMEOUT_MS", timeout);
    cpuShutdownOkWatchdogTimer.expires_after(
        std::chrono::milliseconds(timeout));
    cpuShutdownOkWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "CPU shutdown OK watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("CPU shutdown OK watchdog timer canceled");
            return;
        }
        lg2::info("CPU shutdown OK watchdog timer expired");
        sendPowerControlEvent(Event::cpuShutdownOkWatchdogTimerExpired);
    });
}

static void hpmPowerGoodWatchdogTimerStart(int timeoutMs = -1)
{
    // Use provided timeout or default from TimerMap
    int timeout = (timeoutMs > 0) ? timeoutMs : TimerMap["HpmPowerGoodWatchdogMs"];
    
    lg2::info("HPM power good watchdog timer started with {TIMEOUT_MS}ms timeout",
              "TIMEOUT_MS", timeout);
    hpmPowerGoodWatchdogTimer.expires_after(
        std::chrono::milliseconds(timeout));
    hpmPowerGoodWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "HPM power good watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("HPM power good watchdog timer canceled");
            return;
        }
        lg2::info("HPM power good watchdog timer expired");
        sendPowerControlEvent(Event::hpmPowerGoodWatchdogTimerExpired);
    });
}

static void warmResetCheckTimerStart()
{
    lg2::info("Warm reset check timer started");
    warmResetCheckTimer.expires_after(
        std::chrono::milliseconds(TimerMap["WarmResetCheckMs"]));
    warmResetCheckTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Warm reset check async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("Warm reset check timer canceled");
            return;
        }
        lg2::info("Warm reset check timer completed");
        sendPowerControlEvent(Event::warmResetDetected);
    });
}

static void pohCounterTimerStart()
{
    lg2::info("POH timer started");
    // Set the time-out as 1 hour, to align with POH command in ipmid
    pohCounterTimer.expires_after(std::chrono::hours(1));
    pohCounterTimer.async_wait([](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("POH timer async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("POH timer canceled");
            return;
        }

        if (getHostState(powerState) !=
            "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return;
        }

        conn->async_method_call(
            [](boost::system::error_code ec,
               const std::variant<uint32_t>& pohCounterProperty) {
                if (ec)
                {
                    lg2::error("error getting poh counter");
                    return;
                }
                const uint32_t* pohCounter =
                    std::get_if<uint32_t>(&pohCounterProperty);
                if (pohCounter == nullptr)
                {
                    lg2::error("unable to read poh counter");
                    return;
                }

                conn->async_method_call(
                    [](boost::system::error_code ec) {
                        if (ec)
                        {
                            lg2::error("failed to set poh counter");
                        }
                    },
                    "xyz.openbmc_project.Settings",
                    "/xyz/openbmc_project/state/chassis0",
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.PowerOnHours", "POHCounter",
                    std::variant<uint32_t>(*pohCounter + 1));
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/state/chassis0",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.State.PowerOnHours", "POHCounter");

        pohCounterTimerStart();
    });
}

static void currentHostStateMonitor()
{
    if (getHostState(powerState) ==
        "xyz.openbmc_project.State.Host.HostState.Running")
    {
        pohCounterTimerStart();
        // Clear the restart cause set for the next restart
        clearRestartCause();
    }
    else
    {
        pohCounterTimer.cancel();
        // Set the restart cause set for this restart
        setRestartCause();
    }

    static auto match = sdbusplus::bus::match_t(
        *conn,
        "type='signal',member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "arg0='xyz.openbmc_project.State.Host'",
        [](sdbusplus::message_t& message) {
            std::string intfName;
            std::map<std::string, std::variant<std::string>> properties;

            try
            {
                message.read(intfName, properties);
            }
            catch (const std::exception& e)
            {
                lg2::error("Unable to read host state: {ERROR}", "ERROR", e);
                return;
            }
            if (properties.empty())
            {
                lg2::error("ERROR: Empty PropertiesChanged signal received");
                return;
            }

            // We only want to check for CurrentHostState
            if (properties.begin()->first != "CurrentHostState")
            {
                return;
            }
            std::string* currentHostState =
                std::get_if<std::string>(&(properties.begin()->second));
            if (currentHostState == nullptr)
            {
                lg2::error("{PROPERTY} property invalid", "PROPERTY",
                           properties.begin()->first);
                return;
            }

            if (*currentHostState ==
                "xyz.openbmc_project.State.Host.HostState.Running")
            {
                pohCounterTimerStart();
                // Clear the restart cause set for the next restart
                clearRestartCause();
                sd_journal_send("MESSAGE=Host system DC power is on",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOn", NULL);
            }
            else
            {
                pohCounterTimer.cancel();
                // POST_COMPLETE GPIO event is not working in some platforms
                // when power state is changed to OFF. This resulted in
                // 'OperatingSystemState' to stay at 'Standby', even though
                // system is OFF. Set 'OperatingSystemState' to 'Inactive'
                // if HostState is trurned to OFF.
                setOperatingSystemState(OperatingSystemStateStage::Inactive);

                // Set the restart cause set for this restart
                setRestartCause();
#ifdef USE_ACBOOT
                resetACBootProperty();
#endif // USE_ACBOOT
                sd_journal_send("MESSAGE=Host system DC power is off",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOff", NULL);
            }
        });
}

static void sioPowerGoodWatchdogTimerStart()
{
    lg2::info("SIO power good watchdog timer started");
    sioPowerGoodWatchdogTimer.expires_after(
        std::chrono::milliseconds(TimerMap["SioPowerGoodWatchdogMs"]));
    sioPowerGoodWatchdogTimer.async_wait([](const boost::system::error_code
                                                ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "SIO power good watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("SIO power good watchdog timer canceled");
            return;
        }
        lg2::info("SIO power good watchdog timer completed");
        sendPowerControlEvent(Event::sioPowerGoodWatchdogTimerExpired);
    });
}

static void powerStateOn(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        case Event::sioS5Assert:
            setPowerState(PowerState::transitionToOff);
#if IGNORE_SOFT_RESETS_DURING_POST
            // Only recognize soft resets once host gets past POST COMPLETE
            if (operatingSystemState != OperatingSystemStateStage::Standby)
            {
                ignoreNextSoftReset = true;
            }
#endif
            addRestartCause(RestartCause::softReset);
            break;
#if USE_PLT_RST
        case Event::pltRstAssert:
#else
        case Event::postCompleteDeAssert:
#endif
            setPowerState(PowerState::checkForWarmReset);
#if IGNORE_SOFT_RESETS_DURING_POST
            // Only recognize soft resets once host gets past POST COMPLETE
            if (operatingSystemState != OperatingSystemStateStage::Standby)
            {
                ignoreNextSoftReset = true;
            }
#endif
            addRestartCause(RestartCause::softReset);
            warmResetCheckTimerStart();
            break;
        case Event::powerButtonPressed:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            break;
        case Event::powerOffRequest:
            lg2::info("Host Force Off Request received. Commencing Host Force Off sequence.");

            // Set the power context for the power on sequence
            detectBoardPresence();
            powerContext.action = PowerAction::FORCE_OFF;
            powerContext.target_state = "HostPowerOff";

            // Check if no boards or no PDB present - error condition
            if ((!powerContext.presence.board0 && !powerContext.presence.board1))
            {
                lg2::error("Force off requested but no boards present. Cannot proceed with force off.");
                // TODO: Handle error appropriately - set error state or fault condition
            }
            // Check if all present boards already have their power rails off
            // For each board: if not present OR (present AND power is de-asserted)
            // This ensures we only check get_value() on boards that are present
            else if ((!powerContext.presence.nvl144_pdb || (nvl144pdbMainPowerOkLine.get_value() == !nvl144pdbMainPowerOkConfig.polarity)) &&
                     (!powerContext.presence.c2_pdb || (c2pdbPSUPowerOkLine.get_value() == !c2pdbPSUPowerOkConfig.polarity)) &&
                     (!powerContext.presence.board0 || (board0RunPowerPGLine.get_value() == !board0RunPowerPGConfig.polarity)) &&
                     (!powerContext.presence.board1 || (board1RunPowerPGLine.get_value() == !board1RunPowerPGConfig.polarity)))
            {
                lg2::info("All present boards have their Main Power Rails Disabled. Main power is already Off. Transitioning to Host Power Off state.");
                // To-Do: Should I set the power enable signals to de-asserted? (to ensure consistency with power state)
                setPowerState(PowerState::off);
            
            }
            else // Main Power Rails are not Off, commence Force Off sequence
            {
                if (powerContext.presence.nvl144_pdb)
                {
                    // NVL144 PDB Main Power OK is de-asserted
                    if (nvl144pdbMainPowerOkLine.get_value() == !nvl144pdbMainPowerOkConfig.polarity)
                    {
                        lg2::info("NVL144 PDB Main Power OK is already de-asserted. Ensuring PDB Main Power Enable is de-asserted. Asserting Board 0's CPU Shutdown Request Line. Waiting for Board 0 & Board 1 CPU Shutdown OK Assertion Events...");

                        // De-assert NVL144 PDB Main Power Enable
                        setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);

                        // Begin HPM Board Power Sequencing. 
                        // Assert Board 0's CPU Shutdown Request Line (for 1P & 2P configs), start CPU Shutdown OK watchdog timer, and wait for CPU Shutdown OK Assertion Events
                        if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                        {
                            setGPIOOutput(board0CpuShutdownForceConfig.lineName, board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                        }

                        cpuShutdownOkWatchdogTimerStart(100); // 100ms timeout for force off sequence
                        setPowerState(PowerState::waitForCPUShutdownOk);
                    }
                    else // NVL144 PDB Main Power OK is asserted
                    {
                        lg2::info("De-asserting NVL144 PDB Main Power Enable. Waiting for PDB Main Power OK De-assertion Event...");

                        // De-assert NVL144 PDB Main Power Enable, start PDB Main Power OK watchdog timer, and wait for PDB Main Power OK De-assertion Event
                        setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
                        pdbMainPowerOkWatchdogTimerStart();
                        setPowerState(PowerState::waitForPDBMainPowerOff);
                    }
                }
                else // C2 PDB or No PDB Sequence Start
                {
                    lg2::info("Commencing HPM Board Power Sequencing. Asserting Board 0's CPU Shutdown Request Line. Waiting for Board 0 & Board 1 CPU Shutdown OK Assertion Events...");

                    // Begin HPM Board Power Sequencing. 
                    // Assert Board 0's CPU Shutdown Request Line (for 1P & 2P configs), start CPU Shutdown OK watchdog timer, and wait for CPU Shutdown OK Assertion Events
                    if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board0CpuShutdownForceConfig.lineName, board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                    }

                    cpuShutdownOkWatchdogTimerStart(100);
                    setPowerState(PowerState::waitForCPUShutdownOk);
                }
            }
            break;
        case Event::gracefulPowerOffRequest:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::powerCycleRequest:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerCycleRequest:
            setPowerState(PowerState::gracefulTransitionToCycleOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::resetButtonPressed:
            setPowerState(PowerState::checkForWarmReset);
            warmResetCheckTimerStart();
            break;
        case Event::resetRequest:
            reset();
            break;
        case Event::board0CpuShutdownOkAssert: case Event::board1CpuShutdownOkAssert:
            // // handle Host Initiated Shutdown Requests
            // lg2::info("Host Initiated Shutdown Request received. Commencing Host Shutdown sequence.");

            // // Set the power context for the power on sequence
            // detectBoardPresence();
            // powerContext.action = PowerAction::HOST_INITIATED_SHUTDOWN;
            // powerContext.target_state = "HostPowerOff";

            
            // To-Do: Handle Host Initiated Shutdown Requests (transition to waitForCPUShutdownOk for both CPUs to assert Shutdown Ok, or begin power off sequence?)
            lg2::warning("Host Initiated Shutdown Request not currently supported yet. Keeping Host Power State at On.");
            break;
        case Event::board0RunPowerPGDeAssert:
            detectBoardPresence();
            if (!powerContext.presence.board1)
            {
                lg2::info("1P Configuration detected. Board 0's Run Power Good de-asserted unexpectedly. Setting Host Power State to Off...");
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else if(powerContext.presence.board1 && board1RunPowerPGLine.get_value() == !board1RunPowerPGConfig.polarity)
            {
                lg2::info("2P Configuration detected.Board 0's Run Power Good de-asserted unexpectedly and Board 1's Run Power Good is also de-asserted. Setting Host Power State to Off...");
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else
            {
                lg2::error("2P Configuration detected. Board 0's Run Power Good de-asserted unexpectedly and Board 1's Run Power Good is asserted. Keeping Host Power State at On.");
            }
            break;
        case Event::board1RunPowerPGDeAssert:
            detectBoardPresence();
            if (!powerContext.presence.board0)
            {
                lg2::info(" 1P Configuration detected. Board 1's Run Power Good de-asserted unexpectedly. Setting Host Power State to Off...");
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else if(powerContext.presence.board0 && board0RunPowerPGLine.get_value() == !board0RunPowerPGConfig.polarity)
            {
                lg2::info("2P Configuration detected. Board 1's Run Power Good de-asserted unexpectedly and Board 0's Run Power Good is also de-asserted. Setting Host Power State to Off...");
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else
            {
                lg2::error("2P Configuration detected. Board 1's Run Power Good de-asserted unexpectedly and Board 0's Run Power Good is asserted. Keeping Host Power State at On.");
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForPSPowerOK(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            psPowerOKWatchdogTimer.cancel();
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::psPowerOKWatchdogTimerExpired:
            setPowerState(PowerState::off);
            psPowerOKFailedLog();
            break;
        case Event::sioPowerGoodAssert:
            psPowerOKWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForSIOPowerGood(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioPowerGoodAssert:
            sioPowerGoodWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            setPowerState(PowerState::off);
            systemPowerGoodFailedLog();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::sioS5DeAssert:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::sioPowerGoodAssert:
            setPowerState(PowerState::on);
            break;
        case Event::powerButtonPressed:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            lg2::info("Power On Request received. Commencing Host Main Power On sequence.");

            // Set the power context for the power on sequence
            detectBoardPresence();
            powerContext.action = PowerAction::POWER_ON;
            powerContext.target_state = "HostPowerOn";
            // Check if all present boards already have their power rails on
            // For each board: if not present OR (present AND power is good)
            // This ensures we only check get_value() on boards that are present
            if ((powerContext.presence.nvl144_pdb || powerContext.presence.c2_pdb || 
                 powerContext.presence.board0 || powerContext.presence.board1) &&
                (!powerContext.presence.nvl144_pdb || (nvl144pdbMainPowerOkLine.get_value() == nvl144pdbMainPowerOkConfig.polarity)) &&
                (!powerContext.presence.c2_pdb || (c2pdbPSUPowerOkLine.get_value() == c2pdbPSUPowerOkConfig.polarity)) &&
                (!powerContext.presence.board0 || (board0RunPowerPGLine.get_value() == board0RunPowerPGConfig.polarity)) &&
                (!powerContext.presence.board1 || (board1RunPowerPGLine.get_value() == board1RunPowerPGConfig.polarity)))
            {
                lg2::info("All present boards have their Main Power Rails Enabled. Main power is already On. Transitioning to Host Power On state.");
                setPowerState(PowerState::on);
            }
            else // Main Power Rails are not On, commence Power On sequence
            {
                // NVL144 PDB Sequence Start
                if(powerContext.presence.nvl144_pdb)
                {
                    // NVL144 PDB Main Power OK is already asserted
                    if (nvl144pdbMainPowerOkLine.get_value() == nvl144pdbMainPowerOkConfig.polarity)
                    {
                        lg2::info("NVL144 PDB Main Power OK is already asserted. Ensuring PDB Main Power Enable is asserted. Commencing HPM Board Power Sequencing.");

                        if (!nvl144pdbMainPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
                        }
                        // Begin HPM Board Power Sequencing. Assert Board 0 and/or Board 1 Pre System Reset
                        if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                        {
                            setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                        }
                        if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                        {
                            setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                        }

                        // Assert Board 0 and/or Board 1 Run Power Enable
                        if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                        }
                        if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                        }

                        // Start the HPM Power Good Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                        hpmPowerGoodWatchdogTimerStart();
                        setPowerState(PowerState::waitForHPMPowerGoodAssert);
                    }
                    else // NVL144 PDB Main Power OK is not asserted, commence NVL144 PDB Main Power On sequence
                    {
                        // Assert NVL144 PDB Main Power Enable
                        lg2::info("Asserting PDB Main Power Enable. Waiting for PDB Main Power OK Assertion Event...");
                        setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);

                        // start the PDB Main Power Ok Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                        pdbMainPowerOkWatchdogTimerStart();
                        setPowerState(PowerState::waitForPDBMainPowerOk);
                    }
                }
                // C2 PDB Sequence Start
                else if(powerContext.presence.c2_pdb)
                {
                    // C2 PDB Main Power OK is already asserted
                    if(c2pdbPSUPowerOkLine.get_value() == c2pdbPSUPowerOkConfig.polarity)
                    {
                        // Enable C2 PDB 12V Rails
                        lg2::info("C2 PDB Main Power OK is already asserted. Ensuring C2 PDB Main Power Enable is asserted. Asserting 12V PSU Enable Lines.");
                        setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
                        setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
                        setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);
                        setGPIOOutput(c2pdb_12V_AICEnableConfig.lineName, c2pdb_12V_AICEnableConfig.polarity, c2pdb_12V_AICEnableLine);

                        lg2::info("Commencing HPM Board Power Sequencing. Asserting HPM BoardPre System Reset & Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");

                        // Begin HPM Board Power Sequencing. Assert Board 0 and/or Board 1 Pre System Reset
                        if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                        {
                            setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                        }
                        if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                        {
                            setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                        }

                        // Assert Board 0 and/or Board 1 Run Power Enable
                        if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                        }
                        if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                        }

                        // Start the HPM Power Good Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                        hpmPowerGoodWatchdogTimerStart();
                        setPowerState(PowerState::waitForHPMPowerGoodAssert);
                    }
                    else // C2 PDB Main Power OK is not asserted, commence C2 PDB Main Power On sequence
                    {
                        // Assert C2 PDB Main Power Enable
                        setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);

                        // start the C2 PDB Main Power Ok Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                        pdbMainPowerOkWatchdogTimerStart();
                        setPowerState(PowerState::waitForPDBMainPowerOk);
                    }
                    
                }
                // No PDB present - HPM Board sequence Start
                else
                {
                    // Start HPM Board Power Sequencing. Assert Board 0 and/or Board 1 Pre System Reset
                    lg2::info("No PDB present. Commencing HPM Board Power Sequencing. Asserting HPM Board Pre System Reset & Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    // Assert Board 0 and/or Board 1 Run Power Enable
                    if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                    {
                        setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                    }
                    if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                    {
                        setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                    }

                    // Start the HPM Power Good Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                    hpmPowerGoodWatchdogTimerStart();
                    setPowerState(PowerState::waitForHPMPowerGoodAssert);
                }
            }

            break;
        case Event::board0RunPowerPGAssert:
            detectBoardPresence();
            if (!powerContext.presence.board1)
            {
                lg2::info("1P Configuration detected. Board 0's Run Power Good asserted unexpectedly. Setting Host Power State to On...");
                setPowerState(PowerState::on);
                powerContext.action = PowerAction::NONE;
            }
            else if(powerContext.presence.board1 && board1RunPowerPGLine.get_value() == board1RunPowerPGConfig.polarity)
            {
                lg2::info("2P Configuration detected.Board 0's Run Power Good asserted unexpectedly and Board 1's Run Power Good is also asserted. Setting Host Power State to On...");
                setPowerState(PowerState::on);
                powerContext.action = PowerAction::NONE;
            }
            else
            {
                lg2::error("2P Configuration detected. Board 0's Run Power Good asserted unexpectedly and Board 1's Run Power Good is de-asserted. Keeping Host Power State at Off.");
            }
            break;
        case Event::board1RunPowerPGAssert:
            detectBoardPresence();
            if (!powerContext.presence.board0)
            {
                lg2::info(" 1P Configuration detected. Board 1's Run Power Good asserted unexpectedly. Setting Host Power State to On...");
                setPowerState(PowerState::on);
                powerContext.action = PowerAction::NONE;
            }
            else if(powerContext.presence.board0 && board0RunPowerPGLine.get_value() == board0RunPowerPGConfig.polarity)
            {
                lg2::info("2P Configuration detected. Board 1's Run Power Good asserted unexpectedly and Board 0's Run Power Good is also asserted. Setting Host Power State to On...");
                setPowerState(PowerState::on);
                powerContext.action = PowerAction::NONE;
            }
            else
            {
                lg2::error("2P Configuration detected. Board 1's Run Power Good asserted unexpectedly and Board 0's Run Power Good is de-asserted. Keeping Host Power State at Off.");
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::off);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::off);
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        case Event::powerOffRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::powerCycleRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::resetRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::on);
            reset();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            powerCycleTimer.cancel();
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::sioS5DeAssert:
            powerCycleTimer.cancel();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerButtonPressed:
            powerCycleTimer.cancel();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerCycleTimerExpired:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        case Event::powerOffRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::powerCycleRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::resetRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::on);
            reset();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateCheckForWarmReset(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioS5Assert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            break;
        case Event::warmResetDetected:
            setPowerState(PowerState::on);
            break;
        case Event::psPowerOKDeAssert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForPDBMainPowerOk(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::nvl144pdbMainPowerOkAssert: // NVL144 PDB
            if(powerContext.action == PowerAction::POWER_ON && powerContext.presence.nvl144_pdb) // Host Main Power On sequence
            {
                // HPM Board Power Sequencing - Begin
                pdbMainPowerOkWatchdogTimer.cancel(); // Cancel the PDB Main Power OK watchdog timer
                lg2::info("Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset and Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");
                
                // Assert Board 0 and/or Board 1 Pre System Reset
                if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                }
                if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                }

                // Assert Board 0 and/or Board 1 Run Power Enable
                if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                }
                if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                }

                // Start the HPM Power Good Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                hpmPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForHPMPowerGoodAssert);
            }
            break;
        case Event::c2pdbPSUPowerOkAssert: // C2 PDB
            if(powerContext.action == PowerAction::POWER_ON && powerContext.presence.c2_pdb)
            {
                pdbMainPowerOkWatchdogTimer.cancel(); // Cancel the PDB Main Power OK watchdog timer
                lg2::info("Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset and Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");

                // Assert Board 0 and/or Board 1 Pre System Reset
                if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                }
                if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                }

                // Assert Board 0 and/or Board 1 Run Power Enable
                if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                }
                if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                }

                // Start the HPM Power Good Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                hpmPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForHPMPowerGoodAssert);
            }
            break;
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                lg2::info("PDB Main Power OK watchdog timer expired. PDB Main Power Sequence Failed. Host Main Power On sequence failed. Conducting Cleanup Sequence: De-asserting PDB Main Power Enable Line. Setting Host Power State to Off.");
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE;

                if(powerContext.presence.nvl144_pdb)
                {
                    setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
                }
                else if(powerContext.presence.c2_pdb)
                {
                    setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, !c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);
                }
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

// To-Do: See if there is refactoring possible to consolidate the nvl144pdbMainPowerOkDeAssert, c2pdbPSUPowerOkDeAssert, and pdbMainPowerOkWatchdogTimerExpired
// events for Host Power On Action since they conduct the same actions.
static void powerStateWaitForPDBMainPowerOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::nvl144pdbMainPowerOkDeAssert:
            pdbMainPowerOkWatchdogTimer.cancel(); // Cancel the PDB Main Power OK watchdog timer
            if(powerContext.action == PowerAction::POWER_ON)
            {
                // NVL144 PDB Main Power Rail Powered Down Successfully
                // Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Set Host Power State to Off.
               lg2::info("PDB Main Power OK De-asserted. NVL144 PDB Main Power Rail Powered Down Successfully! Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Setting Host Power State to Off.");  

               if(powerContext.presence.board0)
               {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                    
               }
               if(powerContext.presence.board1)
               {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                    
               }
               setPowerState(PowerState::off);
               powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // NVL144 PDB Main Power Rail Powered Down Successfully.
                // Commencing HPM Main Power Off sequence. Asserting Board 0's CPU Shutdown Request Line. Waiting for Board 0 & Board 1 CPU Shutdown OK Assertion Events...
                lg2::info("PDB Main Power OK De-asserted. NVL144 PDB Main Power Rail Powered Down Successfully. Commencing HPM Main Power Off sequence. Asserting Board 0's CPU Shutdown Force Line. Waiting for Board 0 & Board 1 CPU Shutdown OK Assertion Events...");
                
                // Assert Board 0's CPU Shutdown Force Line (for 1P & 2P configs), start CPU Shutdown OK watchdog timer, and wait for CPU Shutdown OK Assertion Events
                if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                {
                    setGPIOOutput(board0CpuShutdownForceConfig.lineName, board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                }

                cpuShutdownOkWatchdogTimerStart(100);
                setPowerState(PowerState::waitForCPUShutdownOk);
            }
            break;
        case Event::c2pdbPSUPowerOkDeAssert:
            pdbMainPowerOkWatchdogTimer.cancel(); // Cancel the PDB Main Power OK watchdog timer
            if(powerContext.action == PowerAction::POWER_ON)
            {
                // C2 PDB Main Power Rail Powered Down Successfully.
                // Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Set Host Power State to Off.
               lg2::info("PDB PSU Power OK De-asserted. C2 PDB Main Power Rail Powered Down Successfully. Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Setting Host Power State to Off.");

               if(powerContext.presence.board0)
               {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
               }
               if(powerContext.presence.board1)
               {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
               }
               setPowerState(PowerState::off);
               powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // C2 PDB Main Power Rail Powered Down Successfully. Setting Host Power State to Off.
                lg2::info("PDB PSU Power OK De-asserted. C2 PDB Main Power Rail Powered Down Successfully. Main Power Off sequence completed successfully. Setting Host Power State to Off.");
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state

                // To-Do: Confirm that this is correct for C2 PDB
            }
            break;
        case Event::pdbMainPowerOkWatchdogTimerExpired:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                // Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Set Host Power State to Off.
                lg2::error("Failed to Power Down PDB Main Power Rail. PDB & HPM power domain inconsistency. Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Setting Host Power State to Off.");
                if(powerContext.presence.board0)
                {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                        setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                }
                if(powerContext.presence.board1)
                {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                        setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                }
                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            else if(powerContext.action == PowerAction::FORCE_OFF)
            {
                if(powerContext.presence.nvl144_pdb)
                {
                    // Ensuring de-assertion of NVL144 PDB Main Power Enable. Set Host Power State to Off.
                    lg2::info("PDB Main Power OK watchdog timer expired. Ensuring de-assertion of NVL144 PDB Main Power Enable. Commencing HPM Main Power Off sequence. Asserting Board 0's CPU Shutdown Force Line. Waiting for Board 0 & Board 1 CPU Shutdown OK Assertion Events...");
                    setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);

                    if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board0CpuShutdownForceConfig.lineName, board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                    }

                    cpuShutdownOkWatchdogTimerStart(100);
                    setPowerState(PowerState::waitForCPUShutdownOk);
                }
                else if(powerContext.presence.c2_pdb)
                {
                    // Ensuring de-assertion of C2 PDB 12V Rails and PSU Power Enable. Set Host Power State to Off.
                    lg2::info("PDB Main Power OK watchdog timer expired. Ensuring de-assertion of C2 PDB 12V Rails and PSU Power Enable. Main Power Off sequence failed. Setting Host Power State to Off.");
                    setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, !c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
                    setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, !c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
                    setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, !c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);
                    setGPIOOutput(c2pdb_12V_AICEnableConfig.lineName, !c2pdb_12V_AICEnableConfig.polarity, c2pdb_12V_AICEnableLine);
                    setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, !c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);
                    setPowerState(PowerState::off);
                    powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
                }
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForHPMPowerGoodAssert(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::board0RunPowerPGAssert:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                // If Board 1 is not present, proceed with just Board 0
                if (!powerContext.presence.board1)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 0 Run Power Good Asserted. De-asserting HPM Board 0 Pre System Reset Lines. Waiting for CPU Reset De-assertion...");
            
                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
        
                    // Start the CPU Reset Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetDeAssert);
                }
                // If Board 1 is present and its Run Power Good is asserted, de-assert both Board 0 and Board 1 Pre System Reset Lines and start the CPU Reset Watchdog Timer
                else if(powerContext.presence.board1 && board1RunPowerPGLine.get_value() == board1RunPowerPGConfig.polarity)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 0 & Board 1 Run Power Good Asserted. De-asserting HPM Board 0 Pre System Reset Lines. Waiting for CPU Reset De-assertion...");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }

                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }
        
                    // Start the CPU Reset Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetDeAssert);
                }
                // If Board 1 is present but not powered on yet, wait and let hpmPowerGoodWatchdogTimer continue...
                else
                {
                    lg2::info("HPM Board 0 Run Power Good Asserted. Waiting for Board 1 Run Power Good Assertion...");
                }
            }
            break;

        case Event::board1RunPowerPGAssert:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                // If Board 0 is not present, proceed with just Board 1
                if (!powerContext.presence.board0)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 1 Run Power Good Asserted. De-asserting HPM Board 0 Pre System Reset Lines. Waiting for CPU Reset De-assertion...");
                    
                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }
        
                    // Start the CPU Reset Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetDeAssert);
                }
                // If Board 0 is present and its Run Power Good is asserted, de-assert both Board 0 and Board 1 Pre System Reset Lines and start the CPU Reset Watchdog Timer
                else if(powerContext.presence.board0 && board0RunPowerPGLine.get_value() == board0RunPowerPGConfig.polarity)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 0 & Board 1 Run Power Good Asserted. De-asserting HPM Board 0 Pre System Reset Lines. Waiting for CPU Reset De-assertion...");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }

                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }
        
                    // Start the CPU Reset Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetDeAssert);
                }
                // If Board 0 is present but not powered on yet, wait and let hpmPowerGoodWatchdogTimer continue...
                else
                {
                    lg2::info("HPM Board 1 Run Power Good Asserted. Waiting for Board 0 Run Power Good Assertion...");
                }
            }
            break;

        case Event::hpmPowerGoodWatchdogTimerExpired:
            if(powerContext.action == PowerAction::POWER_ON)
            {
                lg2::error("HPM Main Power On Fault detected. Main Power On sequence failed.");

                if(powerContext.presence.nvl144_pdb)
                {
                    lg2::info("Powering down PDB Main Power Rail to establish HPM/PDB consistency. De-asserting NVL144 PDB Main Power Enable.");
                    setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
                    pdbMainPowerOkWatchdogTimerStart();
                    setPowerState(PowerState::waitForPDBMainPowerOff);
                }
                else if(powerContext.presence.c2_pdb)
                {
                    lg2::info("Powering down PDB Main Power Rail to establish HPM/PDB consistency. De-asserting C2 PDB 12V Rails and PSU Power Enable.");
                    setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, !c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
                    setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, !c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
                    setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, !c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);
                    setGPIOOutput(c2pdb_12V_AICEnableConfig.lineName, !c2pdb_12V_AICEnableConfig.polarity, c2pdb_12V_AICEnableLine);
                    setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, !c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);
                    pdbMainPowerOkWatchdogTimerStart();
                    setPowerState(PowerState::waitForPDBMainPowerOff);
                    
                }
                else
                {
                    lg2::info("Conducting HPM Main Power On Fault Clean up. De-asserting Pre System Reset & Run Power Enable. Setting Host Power State to Off.");
                    if(powerContext.presence.board0)
                    {
                        setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                        setPowerState(PowerState::off);
                        powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
                    }
                    if(powerContext.presence.board1)
                    {
                        setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                        setPowerState(PowerState::off);
                        powerContext.action = PowerAction::NONE;
                    }
                }
            }
            break;

        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForHPMPowerGoodDeAssert(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::board0RunPowerPGDeAssert:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // If Board 1 is not present, proceed with just Board 0. Host Forceful Shutdown Sequence Completed Successfully. Set Host Power State to Off.
                if (!powerContext.presence.board1)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 0 Run Power Good Asserted. Host Forceful Shutdown sequence completed successfully! Conducting Cleanup Sequence: De-asserting HPM Board 0 Pre System Reset Lines & Shutdown Force Lines. Setting Host Power State to Off!");
            
                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
                    if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                    }

                    setPowerState(PowerState::off);
                    powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
                }
                // If Board 1 is present and its Run Power Good is de-asserted, Host Forceful Shutdown Sequence Completed Successfully. Set Host Power State to Off.
                // Conduct cleanup sequence:de-assert both Board 0 and Board 1 Pre System Reset Lines and Shutdown Force Lines.
                else if(powerContext.presence.board1 && board1RunPowerPGLine.get_value() == !board1RunPowerPGConfig.polarity)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 0 & Board 1 Run Power Good Asserted. Host Forceful Shutdown sequence completed successfully! Conducting Cleanup Sequence: De-asserting HPM Board 0 Pre System Reset Lines & Shutdown Force Lines. Setting Host Power State to Off!");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                    }
                    if(powerContext.presence.board1 && !board1CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine);
                    }

                    setPowerState(PowerState::off);
                    powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
                }
                // If Board 1 is present but not powered on yet, wait and let hpmPowerGoodWatchdogTimer continue...
                else
                {
                    lg2::info("HPM Board 0 Run Power Good Asserted. Waiting for Board 1 Run Power Good Assertion...");
                }
            }
            break;
        case Event::board1RunPowerPGDeAssert:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // If Board 0 is not present, proceed with just Board 1. Host Forceful Shutdown Sequence Completed Successfully. Set Host Power State to Off.
                if (!powerContext.presence.board0)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 1 Run Power Good Asserted. Host Forceful Shutdown sequence completed successfully! Conducting Cleanup Sequence: De-asserting HPM Board 1 Pre System Reset Lines & Shutdown Force Lines. Setting Host Power State to Off!");
            
                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }
                    if(powerContext.presence.board1 && !board1CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine);
                    }

                    setPowerState(PowerState::off);
                    powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
                }
                // If Board 0 is present and its Run Power Good is de-asserted, Host Forceful Shutdown Sequence Completed Successfully. Set Host Power State to Off.
                // Conduct cleanup sequence:de-assert both Board 0 and Board 1 Pre System Reset Lines and Shutdown Force Lines.
                else if(powerContext.presence.board0 && board0RunPowerPGLine.get_value() == !board0RunPowerPGConfig.polarity)
                {
                    hpmPowerGoodWatchdogTimer.cancel(); // Cancel the HPM Power Good watchdog timer
                    lg2::info("HPM Board 0 & Board 1 Run Power Good Asserted. Host Forceful Shutdown sequence completed successfully! Conducting Cleanup Sequence: De-asserting HPM Board 0 Pre System Reset Lines & Shutdown Force Lines. Setting Host Power State to Off!");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                    }
                    if(powerContext.presence.board1 && !board1CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine);
                    }

                    setPowerState(PowerState::off);
                    powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
                }
                else
                {
                    lg2::info("HPM Board 1 Run Power Good Asserted. Waiting for Board 0 Run Power Good Assertion...");
                }
            }
            break;
        case Event::hpmPowerGoodWatchdogTimerExpired:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                    lg2::error("HPM Power Good Watchdog Timer Expired. Host Forceful Shutdown sequence failed! Conducting Cleanup Sequence: De-asserting Pre System Reset & Shutdown Force lines. Setting Host Power State to On.");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                    }
                    if(powerContext.presence.board1 && !board1CpuShutdownForceConfig.lineName.empty())
                    {
                        setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine);
                    }

                    setPowerState(PowerState::on);
                    powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForCPUResetAssert(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::cpuResetIndicatorAssert:
            cpuResetWatchdogTimer.cancel(); // Cancel the CPU Reset Watchdog Timer
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                lg2::info("CPU Reset Indicator Asserted. CPUs are out of reset. De-asserting Run Power Enable Lines. Waiting for HPM Board Power Good Assertion Events...");

                if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                }
                if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                }

                hpmPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForHPMPowerGoodDeAssert);
            }
            break;
        case Event::cpuResetWatchdogTimerExpired:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                lg2::error("CPU Reset Watchdog Timer Expired. CPUs are not in reset. Host Forceful Shutdown sequence failed. Conducting Cleanup Sequence: De-asserting Pre System Reset & Shutdown Force Lines. Setting Host Power State to Off.");
                
                if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                }
                if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                }

                if(powerContext.presence.board0 && !board0CpuShutdownForceConfig.lineName.empty())
                {
                    setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
                }
                if(powerContext.presence.board1 && !board1CpuShutdownForceConfig.lineName.empty())
                {
                    setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine);
                }

                setPowerState(PowerState::off);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForCPUResetDeAssert(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::cpuResetIndicatorDeAssert:
            cpuResetWatchdogTimer.cancel(); // Cancel the CPU Reset Watchdog Timer
            if(powerContext.action == PowerAction::POWER_ON)
            {
                lg2::info("CPU Reset De-asserted. CPUs are out of reset. Powered On Host Successfully. Setting Host Power State to On.");
                setPowerState(PowerState::on);
                powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the On state
            }
            break;
        case Event::cpuResetWatchdogTimerExpired:
            lg2::error("CPU Reset Watchdog Timer Expired. CPUs are not out of reset. Host Power On sequence failed. Conducting Cleanup Sequence: De-asserting HPM Board Run Power Enable. Setting Host Power State to Off.");
            
            if(powerContext.presence.board0)
            {
                setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
            }
            if(powerContext.presence.board1)
            {
                setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
            }   
            setPowerState(PowerState::off);
            powerContext.action = PowerAction::NONE; // Clear the power action as Host reached the Off state
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForCPUShutdownOk(const Event event)
{    
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::board0CpuShutdownOkAssert:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // If Board 1 is not present, proceed with just Board 0
                if (!powerContext.presence.board1)
                {
                    cpuShutdownOkWatchdogTimer.cancel(); // Cancel the CPU Shutdown OK watchdog timer
                    lg2::info("Board 0 CPU Shutdown OK Asserted. Asserting Board 0's Pre System Reset Line. Waiting for CPU Reset Indicator Assertion...");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }
                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetAssert);
                }
                // If Board 1 is present and its Shutdown Ok is asserted, de-assert both Board 0 and Board 1 Pre System Reset Lines and start the CPU Reset Watchdog Timer
                else if(powerContext.presence.board1 && board1CpuShutdownOkLine.get_value() == board1CpuShutdownOkConfig.polarity)
                {
                    cpuShutdownOkWatchdogTimer.cancel(); // Cancel the CPU Shutdown OK watchdog timer
                    lg2::info("Board 0 & Board 1 CPU Shutdown OK Asserted. Asserting Board 0 Pre System Reset Lines. Waiting for CPU Reset Indicator Assertion...");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }

                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetAssert);

                }
                // If Board 1 is present but not powered on yet, wait and let hpmPowerGoodWatchdogTimer continue...
                else
                {
                    lg2::info("Board 0 CPU Shutdown OK Asserted. Waiting for Board 1 CPU Shutdown OK Assertion...");
                }
            }
            break;
        case Event::board1CpuShutdownOkAssert:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // If Board 1 is not present, proceed with just Board 0
                if (!powerContext.presence.board0)
                {
                    cpuShutdownOkWatchdogTimer.cancel(); // Cancel the CPU Shutdown OK watchdog timer
                    lg2::info("Board 1 CPU Shutdown OK Asserted. Asserting Board 1's Pre System Reset Line. Waiting for CPU Reset Indicator Assertion...");

                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetAssert);
                }
                // If Board 1 is present and its Shutdown Ok is asserted, de-assert both Board 0 and Board 1 Pre System Reset Lines and start the CPU Reset Watchdog Timer
                else if(powerContext.presence.board0 && board0CpuShutdownOkLine.get_value() == board0CpuShutdownOkConfig.polarity)
                {
                    cpuShutdownOkWatchdogTimer.cancel(); // Cancel the CPU Shutdown OK watchdog timer
                    lg2::info("Board 0 & Board 1 CPU Shutdown OK Asserted. Asserting Board 0 Pre System Reset Lines. Waiting for CPU Reset Indicator Assertion...");

                    if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                    }

                    if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                    {
                        setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                    }

                    cpuResetWatchdogTimerStart();
                    setPowerState(PowerState::waitForCPUResetAssert);

                }
                // If Board 1 is present but not powered on yet, wait and let hpmPowerGoodWatchdogTimer continue...
                else
                {
                    lg2::info("Board 1 CPU Shutdown OK Asserted. Waiting for Board 0 CPU Shutdown OK Assertion...");
                }
            }
            break;
        case Event::cpuShutdownOkWatchdogTimerExpired:
            if(powerContext.action == PowerAction::FORCE_OFF)
            {
                // log that all CPUs did not assert Shutdown Ok in time. Non-failure 
                // Continue with Force Off sequence. De-assert Board 0 & Board 1 Pre System Reset Lines.
                cpuShutdownOkWatchdogTimer.cancel();
                lg2::warning("CPUs did not assert Shutdown Ok within configured timeout {TIMEOUT_MS}ms. Non-failure. Continuing with Force Off sequence. Asserting Board 0 & Board 1 Pre System Reset Lines. Waiting for CPU Reset Indicator assertion event ...", "TIMEOUT_MS", TimerMap["CpuShutdownOkWatchdogMs"]);

                if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                }
                if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                }
                cpuResetWatchdogTimerStart();
                setPowerState(PowerState::waitForCPUResetAssert);
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void psPowerOKHandler(bool state)
{
    Event powerControlEvent = (state == powerOkConfig.polarity)
                                  ? Event::psPowerOKAssert
                                  : Event::psPowerOKDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void sioPowerGoodHandler(bool state)
{
    Event powerControlEvent = (state == sioPwrGoodConfig.polarity)
                                  ? Event::sioPowerGoodAssert
                                  : Event::sioPowerGoodDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void sioOnControlHandler(bool state)
{
    lg2::info("SIO_ONCONTROL value changed: {VALUE}", "VALUE",
              static_cast<int>(state));
}

static void sioS5Handler(bool state)
{
    Event powerControlEvent = (state == sioS5Config.polarity)
                                  ? Event::sioS5Assert
                                  : Event::sioS5DeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void powerButtonHandler(bool state)
{
    bool asserted = state == powerButtonConfig.polarity;
    powerButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        powerButtonPressLog();
        if (!powerButtonMask)
        {
            sendPowerControlEvent(Event::powerButtonPressed);
            addRestartCause(RestartCause::powerButton);
        }
        else
        {
            lg2::info("power button press masked");
        }
    }
#if USE_BUTTON_PASSTHROUGH
    gpiod::line gpioLine;
    bool outputState =
        asserted ? powerOutConfig.polarity : (!powerOutConfig.polarity);
    if (!setGPIOOutput(powerOutConfig.lineName, outputState, gpioLine))
    {
        lg2::error("{GPIO_NAME} power button passthrough failed", "GPIO_NAME",
                   powerOutConfig.lineName);
    }
#endif
}

static void resetButtonHandler(bool state)
{
    bool asserted = state == resetButtonConfig.polarity;
    resetButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        resetButtonPressLog();
        if (!resetButtonMask)
        {
            sendPowerControlEvent(Event::resetButtonPressed);
            addRestartCause(RestartCause::resetButton);
        }
        else
        {
            lg2::info("reset button press masked");
        }
    }
#if USE_BUTTON_PASSTHROUGH
    gpiod::line gpioLine;
    bool outputState =
        asserted ? resetOutConfig.polarity : (!resetOutConfig.polarity);
    if (!setGPIOOutput(resetOutConfig.lineName, outputState, gpioLine))
    {
        lg2::error("{GPIO_NAME} reset button passthrough failed", "GPIO_NAME",
                   resetOutConfig.lineName);
    }
#endif
}

#ifdef CHASSIS_SYSTEM_RESET
static constexpr auto systemdBusname = "org.freedesktop.systemd1";
static constexpr auto systemdPath = "/org/freedesktop/systemd1";
static constexpr auto systemdInterface = "org.freedesktop.systemd1.Manager";
static constexpr auto systemTargetName = "chassis-system-reset.target";

void systemReset()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("Failed to call chassis system reset: {ERR}", "ERR",
                           ec.message());
            }
        },
        systemdBusname, systemdPath, systemdInterface, "StartUnit",
        systemTargetName, "replace");
}
#endif

static void nmiSetEnableProperty(bool value)
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "Enabled",
        std::variant<bool>{value});
}

static void nmiReset(void)
{
    const static constexpr int nmiOutPulseTimeMs = 200;

    lg2::info("NMI out action");
    nmiOutLine.set_value(nmiOutConfig.polarity);
    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME",
              nmiOutConfig.lineName, "GPIO_VALUE", nmiOutConfig.polarity);
    gpioAssertTimer.expires_after(std::chrono::milliseconds(nmiOutPulseTimeMs));
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        // restore the NMI_OUT GPIO line back to the opposite value
        nmiOutLine.set_value(!nmiOutConfig.polarity);
        lg2::info("{GPIO_NAME} released", "GPIO_NAME", nmiOutConfig.lineName);
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                           "GPIO_NAME", nmiOutConfig.lineName, "ERROR_MSG",
                           ec.message());
            }
        }
    });
    // log to redfish
    nmiDiagIntLog();
    lg2::info("NMI out action completed");
    // reset Enable Property
    nmiSetEnableProperty(false);
}

static void nmiSourcePropertyMonitor(void)
{
    lg2::info("NMI Source Property Monitor");

    static std::unique_ptr<sdbusplus::bus::match_t> nmiSourceMatch =
        std::make_unique<sdbusplus::bus::match_t>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',"
            "member='PropertiesChanged',"
            "arg0namespace='xyz.openbmc_project.Chassis.Control.NMISource'",
            [](sdbusplus::message_t& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<bool, std::string>>
                    propertiesChanged;
                std::string state;
                bool value = true;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    if (propertiesChanged.begin()->first == "Enabled")
                    {
                        value =
                            std::get<bool>(propertiesChanged.begin()->second);
                        lg2::info(
                            "NMI Enabled propertiesChanged value: {VALUE}",
                            "VALUE", value);
                        nmiEnabled = value;
                        if (nmiEnabled)
                        {
                            nmiReset();
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    lg2::error("Unable to read NMI source: {ERROR}", "ERROR",
                               e);
                    return;
                }
            });
}

static void setNmiSource()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "BMCSource",
        std::variant<std::string>{
            "xyz.openbmc_project.Chassis.Control.NMISource.BMCSourceSignal.FrontPanelButton"});
    // set Enable Property
    nmiSetEnableProperty(true);
}

static void nmiButtonHandler(bool state)
{
    // Don't handle event if host not running and config doesn't force it
    if (!nmiWhenPoweredOff &&
        getHostState(powerState) !=
            "xyz.openbmc_project.State.Host.HostState.Running")
    {
        return;
    }

    bool asserted = state == nmiButtonConfig.polarity;
    nmiButtonIface->set_property("ButtonPressed", asserted);
    if (asserted)
    {
        nmiButtonPressLog();
        if (nmiButtonMasked)
        {
            lg2::info("NMI button press masked");
        }
        else
        {
            setNmiSource();
        }
    }
}

static void idButtonHandler(bool state)
{
    bool asserted = state == idButtonConfig.polarity;
    idButtonIface->set_property("ButtonPressed", asserted);
}

static void pltRstHandler(bool pltRst)
{
    if (pltRst)
    {
        sendPowerControlEvent(Event::pltRstDeAssert);
    }
    else
    {
        sendPowerControlEvent(Event::pltRstAssert);
    }
}

[[maybe_unused]] static void hostMiscHandler(sdbusplus::message_t& msg)
{
    std::string interfaceName;
    boost::container::flat_map<std::string, std::variant<bool>>
        propertiesChanged;
    try
    {
        msg.read(interfaceName, propertiesChanged);
    }
    catch (const std::exception& e)
    {
        lg2::error("Unable to read Host Misc status: {ERROR}", "ERROR", e);
        return;
    }
    if (propertiesChanged.empty())
    {
        lg2::error("ERROR: Empty Host.Misc PropertiesChanged signal received");
        return;
    }

    for (auto& [property, value] : propertiesChanged)
    {
        if (property == "ESpiPlatformReset")
        {
            bool* pltRst = std::get_if<bool>(&value);
            if (pltRst == nullptr)
            {
                lg2::error("{PROPERTY} property invalid", "PROPERTY", property);
                return;
            }
            pltRstHandler(*pltRst);
        }
    }
}

static void postCompleteHandler(bool state)
{
    bool asserted = state == postCompleteConfig.polarity;
    if (asserted)
    {
        sendPowerControlEvent(Event::postCompleteAssert);
        setOperatingSystemState(OperatingSystemStateStage::Standby);
    }
    else
    {
        sendPowerControlEvent(Event::postCompleteDeAssert);
        setOperatingSystemState(OperatingSystemStateStage::Inactive);
    }
}

static void nvl144pdbMainPowerOkHandler(bool state)
{
    Event powerControlEvent = (state == nvl144pdbMainPowerOkConfig.polarity)
                                  ? Event::nvl144pdbMainPowerOkAssert
                                  : Event::nvl144pdbMainPowerOkDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void c2pdbPSUPowerOkHandler(bool state)
{
    Event powerControlEvent = (state == c2pdbPSUPowerOkConfig.polarity)
                                  ? Event::c2pdbPSUPowerOkAssert
                                  : Event::c2pdbPSUPowerOkDeAssert;
    sendPowerControlEvent(powerControlEvent);
}
static void board0RunPowerPGHandler(bool state)
{
    Event powerControlEvent = (state == board0RunPowerPGConfig.polarity)
                                  ? Event::board0RunPowerPGAssert
                                  : Event::board0RunPowerPGDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void board1RunPowerPGHandler(bool state)
{
    Event powerControlEvent = (state == board1RunPowerPGConfig.polarity)
                                  ? Event::board1RunPowerPGAssert
                                  : Event::board1RunPowerPGDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void cpuResetIndicatorHandler(bool state)
{
    Event powerControlEvent = (state == cpuResetIndicatorConfig.polarity)
                                  ? Event::cpuResetIndicatorAssert
                                  : Event::cpuResetIndicatorDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void board0CpuShutdownOkHandler(bool state)
{
    Event powerControlEvent = (state == board0CpuShutdownOkConfig.polarity)
                                  ? Event::board0CpuShutdownOkAssert
                                  : Event::board0CpuShutdownOkDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void board1CpuShutdownOkHandler(bool state)
{
    Event powerControlEvent = (state == board1CpuShutdownOkConfig.polarity)
                                  ? Event::board1CpuShutdownOkAssert
                                  : Event::board1CpuShutdownOkDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

// Board presence detection functions
static bool checkIOXPresence(const std::string& ioxPath)
{
    return std::filesystem::exists(ioxPath);
}

static void detectBoardPresence()
{
    // Check presence and update context using paths from build configuration
    powerContext.presence.c2_pdb = checkIOXPresence(C2_PDB_IOX_PATH);
    powerContext.presence.nvl144_pdb = checkIOXPresence(NVL144_PDB_IOX_PATH);
    powerContext.presence.board0 = checkIOXPresence(BOARD0_IOX_PATH);
    powerContext.presence.board1 = checkIOXPresence(BOARD1_IOX_PATH);
    
    // Log detected board presence
    lg2::info("Board presence detection:");
    lg2::info("  C2 PDB ({PATH}): {PRESENT}", "PATH", std::string(C2_PDB_IOX_PATH),
              "PRESENT", powerContext.presence.c2_pdb);
    lg2::info("  NVL144 PDB ({PATH}): {PRESENT}", "PATH", std::string(NVL144_PDB_IOX_PATH),
              "PRESENT", powerContext.presence.nvl144_pdb);
    lg2::info("  Board 0 ({PATH}): {PRESENT}", "PATH", std::string(BOARD0_IOX_PATH),
              "PRESENT", powerContext.presence.board0);
    lg2::info("  Board 1 ({PATH}): {PRESENT}", "PATH", std::string(BOARD1_IOX_PATH),
              "PRESENT", powerContext.presence.board1);
}

static int loadConfigValues()
{
    const std::string configFilePath =
        "/usr/share/x86-power-control/power-config-host" + power_control::node +
        ".json";
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        lg2::error("loadConfigValues: Cannot open config path \'{PATH}\'",
                   "PATH", configFilePath);
        return -1;
    }
    auto jsonData = nlohmann::json::parse(configFile, nullptr, true, true);

    if (jsonData.is_discarded())
    {
        lg2::error("Power config readings JSON parser failure");
        return -1;
    }
    auto gpios = jsonData["gpio_configs"];
    auto timers = jsonData["timing_configs"];

    ConfigData* tempGpioData;

    for (nlohmann::json& gpioConfig : gpios)
    {
        if (!gpioConfig.contains("Name"))
        {
            lg2::error("The 'Name' field must be defined in Json file");
            return -1;
        }

        // Iterate through the powersignal map to check if the gpio json config
        // entry is valid
        std::string gpioName = gpioConfig["Name"];
        auto signalMapIter = powerSignalMap.find(gpioName);
        if (signalMapIter == powerSignalMap.end())
        {
            lg2::error(
                "{GPIO_NAME} is not a recognized power-control signal name",
                "GPIO_NAME", gpioName);
            return -1;
        }

        // assign the power signal name to the corresponding structure reference
        // from map then fillup the structure with coressponding json config
        // value
        tempGpioData = signalMapIter->second;
        tempGpioData->name = gpioName;

        if (!gpioConfig.contains("Type"))
        {
            lg2::error("The \'Type\' field must be defined in Json file");
            return -1;
        }

        std::string signalType = gpioConfig["Type"];
        if (signalType == "GPIO")
        {
            tempGpioData->type = ConfigType::GPIO;
        }
        else if (signalType == "DBUS")
        {
            tempGpioData->type = ConfigType::DBUS;
        }
        else
        {
            lg2::error("{TYPE} is not a recognized power-control signal type",
                       "TYPE", signalType);
            return -1;
        }

        if (tempGpioData->type == ConfigType::GPIO)
        {
            if (gpioConfig.contains("LineName"))
            {
                tempGpioData->lineName = gpioConfig["LineName"];
            }
            else
            {
                lg2::error(
                    "The \'LineName\' field must be defined for GPIO configuration");
                return -1;
            }
            if (gpioConfig.contains("Polarity"))
            {
                std::string polarity = gpioConfig["Polarity"];
                if (polarity == "ActiveLow")
                {
                    tempGpioData->polarity = false;
                }
                else if (polarity == "ActiveHigh")
                {
                    tempGpioData->polarity = true;
                }
                else
                {
                    lg2::error(
                        "Polarity defined but not properly setup. Please only ActiveHigh or ActiveLow. Currently set to {POLARITY}",
                        "POLARITY", polarity);
                    return -1;
                }
            }
            else
            {
                lg2::error("Polarity field not found for {GPIO_NAME}",
                           "GPIO_NAME", tempGpioData->lineName);
                return -1;
            }
        }
        else
        {
            // if dbus based gpio config is defined read and update the dbus
            // params corresponding to the gpio config instance
            for (auto& [key, dbusParamName] : dbusParams)
            {
                if (!gpioConfig.contains(dbusParamName))
                {
                    lg2::error(
                        "The {DBUS_NAME} field must be defined for Dbus configuration ",
                        "DBUS_NAME", dbusParamName);
                    return -1;
                }
            }
            tempGpioData->dbusName =
                gpioConfig[dbusParams[DbusConfigType::name]];
            tempGpioData->path = gpioConfig[dbusParams[DbusConfigType::path]];
            tempGpioData->interface =
                gpioConfig[dbusParams[DbusConfigType::interface]];
            tempGpioData->lineName =
                gpioConfig[dbusParams[DbusConfigType::property]];

            // dbus-based inputs must be active-high.
            tempGpioData->polarity = true;

            // MatchRegex is optional
            auto item = gpioConfig.find("MatchRegex");
            if (item != gpioConfig.end())
            {
                try
                {
                    tempGpioData->matchRegex = std::regex(*item);
                }
                catch (const std::regex_error& e)
                {
                    lg2::error("Invalid MatchRegex for {NAME}: {ERR}", "NAME",
                               gpioName, "ERR", e.what());
                    return -1;
                }
            }
        }
    }

    // read and store the timer values from json config to Timer Map
    for (auto& [key, timerValue] : TimerMap)
    {
        if (timers.contains(key.c_str()))
        {
            timerValue = timers[key.c_str()];
        }
    }

    // If "events_configs" key is not in json config, fallback to null
    auto events = jsonData.value("event_configs",
                                 nlohmann::json(nlohmann::json::value_t::null));
    if (events.is_object())
    {
        nmiWhenPoweredOff = events.value("NMIWhenPoweredOff", true);
    }

    return 0;
}

template <typename T>
static std::optional<T> getMessageValue(sdbusplus::message_t& msg,
                                        const std::string& name)
{
    std::string event;
    std::string thresholdInterface;
    boost::container::flat_map<std::string, std::variant<T>> propertiesChanged;

    msg.read(thresholdInterface, propertiesChanged);
    if (propertiesChanged.empty())
    {
        return std::nullopt;
    }

    event = propertiesChanged.begin()->first;
    if (event.empty() || event != name)
    {
        return std::nullopt;
    }

    return std::get<T>(propertiesChanged.begin()->second);
}

static bool getDbusMsgGPIOState(sdbusplus::message_t& msg,
                                const ConfigData& config, bool& value)
{
    try
    {
        if (config.matchRegex.has_value())
        {
            std::optional<std::string> s =
                getMessageValue<std::string>(msg, config.lineName);
            if (!s.has_value())
            {
                return false;
            }

            std::smatch m;
            value = std::regex_match(s.value(), m, config.matchRegex.value());
        }
        else
        {
            std::optional<bool> v = getMessageValue<bool>(msg, config.lineName);
            if (!v.has_value())
            {
                return false;
            }
            value = v.value();
        }
        return true;
    }
    catch (const std::exception& e)
    {
        lg2::error(
            "exception while reading dbus property \'{DBUS_NAME}\': {ERROR}",
            "DBUS_NAME", config.lineName, "ERROR", e);
        return false;
    }
}

static sdbusplus::bus::match_t dbusGPIOMatcher(
    const ConfigData& cfg, std::function<void(bool)> onMatch)
{
    auto pulseEventMatcherCallback =
        [&cfg, onMatch](sdbusplus::message_t& msg) {
            bool value = false;
            if (!getDbusMsgGPIOState(msg, cfg, value))
            {
                return;
            }
            onMatch(value);
        };

    return sdbusplus::bus::match_t(
        static_cast<sdbusplus::bus_t&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='" +
            cfg.interface + "',path='" + cfg.path + "',sender='" +
            cfg.dbusName + "'",
        std::move(pulseEventMatcherCallback));
}

// D-Bus property read functions
void reschedulePropertyRead(const ConfigData& configData);

int getProperty(const ConfigData& configData)
{
    std::variant<bool> resp;

    try
    {
        auto method = conn->new_method_call(
            configData.dbusName.c_str(), configData.path.c_str(),
            "org.freedesktop.DBus.Properties", "Get");
        method.append(configData.interface.c_str(),
                      configData.lineName.c_str());

        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            lg2::error(
                "Error reading {PROPERTY} D-Bus property on interface {INTERFACE} and path {PATH}",
                "PROPERTY", configData.lineName, "INTERFACE",
                configData.interface, "PATH", configData.path);
            return -1;
        }

        reply.read(resp);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Exception while reading {PROPERTY}: {WHAT}", "PROPERTY",
                   configData.lineName, "WHAT", e.what());
        reschedulePropertyRead(configData);
        return -1;
    }

    auto respValue = std::get_if<bool>(&resp);
    if (!respValue)
    {
        lg2::error("Error: {PROPERTY} D-Bus property is not the expected type",
                   "PROPERTY", configData.lineName);
        return -1;
    }
    return (*respValue);
}

void setInitialValue(const ConfigData& configData, bool initialValue)
{
    if (configData.name == "PowerOk")
    {
        powerState = (initialValue ? PowerState::on : PowerState::off);
        hostIface->set_property("CurrentHostState",
                                std::string(getHostState(powerState)));
    }
    else if (configData.name == "PowerButton")
    {
        powerButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData.name == "ResetButton")
    {
        resetButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData.name == "NMIButton")
    {
        nmiButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData.name == "IdButton")
    {
        idButtonIface->set_property("ButtonPressed", !initialValue);
    }
    else if (configData.name == "PostComplete")
    {
        OperatingSystemStateStage osState =
            (initialValue == postCompleteConfig.polarity
                 ? OperatingSystemStateStage::Standby
                 : OperatingSystemStateStage::Inactive);
        setOperatingSystemState(osState);
    }
    else
    {
        lg2::error("Unknown name {NAME}", "NAME", configData.name);
    }
}

void reschedulePropertyRead(const ConfigData& configData)
{
    auto item = dBusRetryTimers.find(configData.name);

    if (item == dBusRetryTimers.end())
    {
        auto newItem = dBusRetryTimers.insert(
            {configData.name, boost::asio::steady_timer(io)});

        if (!newItem.second)
        {
            lg2::error("Failed to add new timer for {NAME}", "NAME",
                       configData.name);
            return;
        }

        item = newItem.first;
    }

    auto& timer = item->second;
    timer.expires_after(
        std::chrono::milliseconds(TimerMap["DbusGetPropertyRetry"]));
    timer.async_wait([&configData](const boost::system::error_code ec) {
        if (ec)
        {
            lg2::error("Retry timer for {NAME} failed: {MSG}", "NAME",
                       configData.name, "MSG", ec.message());
            dBusRetryTimers.erase(configData.name);
            return;
        }

        int property = getProperty(configData);

        if (property >= 0)
        {
            setInitialValue(configData, (property > 0));
            dBusRetryTimers.erase(configData.name);
        }
    });
}
} // namespace power_control

int main(int argc, char* argv[])
{
    using namespace power_control;

    if (argc > 1)
    {
        node = argv[1];
    }
    lg2::info("Start Chassis power control service for host : {NODE}", "NODE",
              node);

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    // Load GPIO's through json config file
    if (loadConfigValues() == -1)
    {
        lg2::error("Host{NODE}: Error in Parsing...", "NODE", node);
    }
    /* Currently for single host based systems additional busname is added
    with "0" at the end of the name ex : xyz.openbmc_project.State.Host0.
    Going forward for single hosts the old bus name without zero numbering
    will be removed when all other applications adapted to the
    bus name with zero numbering (xyz.openbmc_project.State.Host0). */

    if (node == "0")
    {
        // Request all the dbus names
        conn->request_name(hostDbusName.c_str());
        conn->request_name(chassisDbusName.c_str());
        conn->request_name(osDbusName.c_str());
        conn->request_name(buttonDbusName.c_str());
        conn->request_name(nmiDbusName.c_str());
        conn->request_name(rstCauseDbusName.c_str());
    }

    hostDbusName += node;
    chassisDbusName += node;
    osDbusName += node;
    buttonDbusName += node;
    nmiDbusName += node;
    rstCauseDbusName += node;

    // Request all the dbus names
    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());
    conn->request_name(osDbusName.c_str());
    conn->request_name(buttonDbusName.c_str());
    conn->request_name(nmiDbusName.c_str());
    conn->request_name(rstCauseDbusName.c_str());

    if (sioPwrGoodConfig.lineName.empty() ||
        sioOnControlConfig.lineName.empty() || sioS5Config.lineName.empty())
    {
        sioEnabled = false;
        lg2::info("SIO control GPIOs not defined, disable SIO support.");
    }

    // Detect Boards Presence and update context
    detectBoardPresence();

    // Request PS_PWROK GPIO events
    // if (powerOkConfig.type == ConfigType::GPIO)
    // {
    //     if (!requestGPIOEvents(powerOkConfig.lineName, psPowerOKHandler,
    //                            psPowerOKLine, psPowerOKEvent))
    //     {
    //         return -1;
    //     }
    // }
    // else if (powerOkConfig.type == ConfigType::DBUS)
    // {
    //     static sdbusplus::bus::match_t powerOkEventMonitor =
    //         power_control::dbusGPIOMatcher(powerOkConfig, psPowerOKHandler);
    // }
    // else
    // {
    //     lg2::error("PowerOk name should be configured from json config file");
    //     return -1;
    // }

    if (sioEnabled == true)
    {
        // Request SIO_POWER_GOOD GPIO events
        if (sioPwrGoodConfig.type == ConfigType::GPIO)
        {
            if (!requestGPIOEvents(sioPwrGoodConfig.lineName,
                                   sioPowerGoodHandler, sioPowerGoodLine,
                                   sioPowerGoodEvent))
            {
                return -1;
            }
        }
        else if (sioPwrGoodConfig.type == ConfigType::DBUS)
        {
            static sdbusplus::bus::match_t sioPwrGoodEventMonitor =
                power_control::dbusGPIOMatcher(sioPwrGoodConfig,
                                               sioPowerGoodHandler);
        }
        else
        {
            lg2::error(
                "sioPwrGood name should be configured from json config file");
            return -1;
        }

        // Request SIO_ONCONTROL GPIO events
        if (sioOnControlConfig.type == ConfigType::GPIO)
        {
            if (!requestGPIOEvents(sioOnControlConfig.lineName,
                                   sioOnControlHandler, sioOnControlLine,
                                   sioOnControlEvent))
            {
                return -1;
            }
        }
        else if (sioOnControlConfig.type == ConfigType::DBUS)
        {
            static sdbusplus::bus::match_t sioOnControlEventMonitor =
                power_control::dbusGPIOMatcher(sioOnControlConfig,
                                               sioOnControlHandler);
        }
        else
        {
            lg2::error(
                "sioOnControl name should be configured from jsonconfig file\n");
            return -1;
        }

        // Request SIO_S5 GPIO events
        if (sioS5Config.type == ConfigType::GPIO)
        {
            if (!requestGPIOEvents(sioS5Config.lineName, sioS5Handler,
                                   sioS5Line, sioS5Event))
            {
                return -1;
            }
        }
        else if (sioS5Config.type == ConfigType::DBUS)
        {
            static sdbusplus::bus::match_t sioS5EventMonitor =
                power_control::dbusGPIOMatcher(sioS5Config, sioS5Handler);
        }
        else
        {
            lg2::error("sioS5 name should be configured from json config file");
            return -1;
        }
    }

    // Request POWER_BUTTON GPIO events
    // if (powerButtonConfig.type == ConfigType::GPIO)
    // {
    //     if (!requestGPIOEvents(powerButtonConfig.lineName, powerButtonHandler,
    //                            powerButtonLine, powerButtonEvent))
    //     {
    //         return -1;
    //     }
    // }
    // else if (powerButtonConfig.type == ConfigType::DBUS)
    // {
    //     static sdbusplus::bus::match_t powerButtonEventMonitor =
    //         dbusGPIOMatcher(powerButtonConfig,
    //                                        powerButtonHandler);
    // }

    // Request RESET_BUTTON GPIO events
    // if (resetButtonConfig.type == ConfigType::GPIO)
    // {
    //     if (!requestGPIOEvents(resetButtonConfig.lineName, resetButtonHandler,
    //                            resetButtonLine, resetButtonEvent))
    //     {
    //         return -1;
    //     }
    // }
    // else if (resetButtonConfig.type == ConfigType::DBUS)
    // {
    //     static sdbusplus::bus::match_t resetButtonEventMonitor =
    //         power_control::dbusGPIOMatcher(resetButtonConfig,
    //                                        resetButtonHandler);
    // }

    // Request NMI_BUTTON GPIO events
    // if (nmiButtonConfig.type == ConfigType::GPIO)
    // {
    //     if (!nmiButtonConfig.lineName.empty())
    //     {
    //         requestGPIOEvents(nmiButtonConfig.lineName, nmiButtonHandler,
    //                           nmiButtonLine, nmiButtonEvent);
    //     }
    // }
    // else if (nmiButtonConfig.type == ConfigType::DBUS)
    // {
    //     static sdbusplus::bus::match_t nmiButtonEventMonitor =
    //         power_control::dbusGPIOMatcher(nmiButtonConfig, nmiButtonHandler);
    // }

    // Request ID_BUTTON GPIO events
    // if (idButtonConfig.type == ConfigType::GPIO)
    // {
    //     if (!idButtonConfig.lineName.empty())
    //     {
    //         requestGPIOEvents(idButtonConfig.lineName, idButtonHandler,
    //                           idButtonLine, idButtonEvent);
    //     }
    // }
    // else if (idButtonConfig.type == ConfigType::DBUS)
    // {
    //     static sdbusplus::bus::match_t idButtonEventMonitor =
    //         dbusGPIOMatcher(idButtonConfig, idButtonHandler);
    // }

#ifdef USE_PLT_RST
    sdbusplus::bus::match_t pltRstMatch(
        *conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='xyz.openbmc_project.State.Host.Misc'",
        hostMiscHandler);
#endif

    // Request POST_COMPLETE GPIO events
    // if (postCompleteConfig.type == ConfigType::GPIO)
    // {
    //     if (!requestGPIOEvents(postCompleteConfig.lineName, postCompleteHandler,
    //                            postCompleteLine, postCompleteEvent))
    //     {
    //         return -1;
    //     }
    // }
    // else if (postCompleteConfig.type == ConfigType::DBUS)
    // {
    //     static sdbusplus::bus::match_t postCompleteEventMonitor =
    //         power_control::dbusGPIOMatcher(postCompleteConfig,
    //                                        postCompleteHandler);
    // }
    // else
    // {
    //     lg2::error(
    //         "postComplete name should be configured from json config file");
    //     return -1;
    // }

    // Request NVL144 or C2 PDB Main Power Ok GPIO events, if PDBs are present

    // Request PDB_MAIN_POWER_OK GPIO events if NVL144 PDB is present
    if (powerContext.presence.nvl144_pdb  && nvl144pdbMainPowerOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(nvl144pdbMainPowerOkConfig.lineName,
                               nvl144pdbMainPowerOkHandler, nvl144pdbMainPowerOkLine,
                               nvl144pdbMainPowerOkEvent))
        {
            return -1;
        }
    }
    else if (powerContext.presence.c2_pdb  && c2pdbPSUPowerOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(c2pdbPSUPowerOkConfig.lineName,
                               c2pdbPSUPowerOkHandler, c2pdbPSUPowerOkLine,
                               c2pdbPSUPowerOkEvent))
        {
            return -1;
        }
    }

    // Request BOARD0_RUN_POWER_PG GPIO events
    if (powerContext.presence.board0 && board0RunPowerPGConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board0RunPowerPGConfig.lineName,
                               board0RunPowerPGHandler, board0RunPowerPGLine,
                               board0RunPowerPGEvent))
        {
            return -1;
        }
    }

    // Request BOARD1_RUN_POWER_PG GPIO events
    if (powerContext.presence.board1 && board1RunPowerPGConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board1RunPowerPGConfig.lineName,
                               board1RunPowerPGHandler, board1RunPowerPGLine,
                               board1RunPowerPGEvent))
        {
            return -1;
        }
    }

    // Request CPU_RESET_INDICATOR GPIO events
    if (cpuResetIndicatorConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(cpuResetIndicatorConfig.lineName,
                               cpuResetIndicatorHandler, cpuResetIndicatorLine,
                               cpuResetIndicatorEvent))
        {
            return -1;
        }
    }

    // Request BOARD0_CPU_SHUTDOWN_OK GPIO events
    if (powerContext.presence.board0 && board0CpuShutdownOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board0CpuShutdownOkConfig.lineName,
                               board0CpuShutdownOkHandler,
                               board0CpuShutdownOkLine,
                               board0CpuShutdownOkEvent))
        {
            return -1;
        }
    }

    // Request BOARD1_CPU_SHUTDOWN_OK GPIO events
    if (powerContext.presence.board1 && board1CpuShutdownOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(board1CpuShutdownOkConfig.lineName,
                               board1CpuShutdownOkHandler,
                               board1CpuShutdownOkLine,
                               board1CpuShutdownOkEvent))
        {
            return -1;
        }
    }

    // Check HPM Run Power Good lines at startup and set host state accordingly
    if (powerContext.presence.board0 && powerContext.presence.board1)
    {
        // 2P Configuration
        bool board0PGAsserted = (board0RunPowerPGLine.get_value() == board0RunPowerPGConfig.polarity);
        bool board1PGAsserted = (board1RunPowerPGLine.get_value() == board1RunPowerPGConfig.polarity);

        if (board0PGAsserted || board1PGAsserted)
        {
            // At least one board has power good asserted
            lg2::info("2P Configuration: At least one HPM Board Run Power Good is asserted at startup. Asserting Board 0 & 1's  Run Power Enable. Setting Host State to On.");
            
            // Assert both board run power enables
            if (!board0RunPowerEnableConfig.lineName.empty())
            {
                if (!setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine))
                {
                    return -1;
                }
            }
            if (!board1RunPowerEnableConfig.lineName.empty())
            {
                if(!setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine))
                {
                    return -1;
                }
            }
            
            // Set host state to On
            powerState = PowerState::on;
            operatingSystemState = OperatingSystemStateStage::Standby;
            
            // Check if only one board is powered on (CPLD fault indication)
            if (board0PGAsserted && !board1PGAsserted)
            {
                lg2::warning("2P Configuration: Board 0 Run Power Good is asserted but Board 1 Run Power Good is not. This may indicate a CPLD fault or system misconfiguration.");
            }
            else if (!board0PGAsserted && board1PGAsserted)
            {
                lg2::warning("2P Configuration: Board 1 Run Power Good is asserted but Board 0 Run Power Good is not. This may indicate a CPLD fault or system misconfiguration.");
            }
        }
        else
        {
            lg2::info("2P Configuration: No HPM Run Power Good is asserted at startup. De-asserting both Board 0 & 1 Run Power Enables. Setting Host State to Off.");

            if(!setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine))
            {
                return -1;
            }
            if(!setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine))
            {
                return -1;
            }

            powerState = PowerState::off;
            operatingSystemState = OperatingSystemStateStage::Inactive;
        }
    }
    else if (powerContext.presence.board0)
    {
        // 1P Configuration (only board0 present)
        bool board0PGAsserted = (board0RunPowerPGLine.get_value() == board0RunPowerPGConfig.polarity);
        
        if (board0PGAsserted)
        {
            lg2::info("1P Configuration: Board 0 HPM Run Power Good is asserted at startup. Asserting Board 0 Run Power Enable. Setting Host State to On.");
            
            // Assert board0 run power enable
            if (!board0RunPowerEnableConfig.lineName.empty())
            {
                if (!setGPIOOutput(board0RunPowerEnableConfig.lineName, board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine))
                {
                    return -1;
                }
            }
            
            // Set host state to On
            powerState = PowerState::on;
            operatingSystemState = OperatingSystemStateStage::Standby;
        }
        else
        {
            lg2::info("1P Configuration: Board 0 HPM Run Power Good is not asserted at startup. De-asserting Board 0 Run Power Enable. Setting Host State to Off.");

            if(!setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine))
            {
                return -1;
            }

            powerState = PowerState::off;
            operatingSystemState = OperatingSystemStateStage::Inactive;
        }
    }
    else if (powerContext.presence.board1)
    {
        // 1P Configuration (only board1 present - unusual but handle it)
        bool board1PGAsserted = (board1RunPowerPGLine.get_value() == board1RunPowerPGConfig.polarity);
        
        if (board1PGAsserted)
        {
            lg2::info("1P Configuration: Board 1 HPM Run Power Good is asserted at startup. Setting Host State to On and asserting Board 1 Run Power Enable.");
            
            // Assert board1 run power enable
            if (!board1RunPowerEnableConfig.lineName.empty())
            {
                setGPIOOutput(board1RunPowerEnableConfig.lineName, board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
            }
            
            // Set host state to On
            powerState = PowerState::on;
            operatingSystemState = OperatingSystemStateStage::Standby;
        }
        else
        {

            if(!setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine))
            {
                return -1;
            }

            lg2::info("1P Configuration: Board 1 HPM Run Power Good is not asserted at startup. Setting Host State to Off.");

            powerState = PowerState::off;
            operatingSystemState = OperatingSystemStateStage::Inactive;
        }
    }

    // initialize output GPIOs:

    if(powerContext.presence.nvl144_pdb)
    {
        if(nvl144pdbMainPowerOkLine.get_value() == nvl144pdbMainPowerOkConfig.polarity)
        {
            if(!setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine))
            {
                return -1;
            }
            lg2::info("NVL144 PDB Main Power OK is asserted at startup. Setting NVL144 PDB Main Power Enable to asserted.");  
        }
        else
        {
            if(!setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine))
            {
                return -1;
            }
            lg2::info("NVL144 PDB Main Power OK is de-asserted at startup. Setting NVL144 PDB Main Power Enable to de-asserted.");
        }
    }

    if(powerContext.presence.c2_pdb)
    {
        //To-Do: Implement C2 PDB Output GPIO Initialization
        // C2PDBPSUPowerEnable, C2PDB_12V_HPMEnable, C2PDB_12V_GPU1Enable, C2PDB_12V_GPU2Enable, C2PDB_12V_AICEnable

    }

    // Board0RunPowerEnable, Board1RunPowerEnable
    // Board0PreSystemReset, Board1PreSystemReset
    // Board0CpuShutdownForce, Board1CpuShutdownForce
    // Board0CpuShutdownRequest, Board1CpuShutdownRequest
    if(powerContext.presence.board0)
    {
        if(!setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine))
        {
            return -1;
        }
        if(!setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine))
        {
            return -1;
        }
        if(!setGPIOOutput(board0CpuShutdownRequestConfig.lineName, !board0CpuShutdownRequestConfig.polarity, board0CpuShutdownRequestLine))
        {
            return -1;
        }
    }

    if(powerContext.presence.board1)
    {
        if(!setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine))
        {
            return -1;
        }
        if(!setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine))
        {
            return -1;
        }
        if(!setGPIOOutput(board1CpuShutdownRequestConfig.lineName, !board1CpuShutdownRequestConfig.polarity, board1CpuShutdownRequestLine))
        {
            return -1;
        }
    }

    if(!setGPIOOutput(usbPowerEnableConfig.lineName, !usbPowerEnableConfig.polarity, usbPowerEnableLine))
    {
        lg2::warning("Failed to set USB Power Enable GPIO to default value. Continuing with power control initialization.");
    }

    // initialize NMI_OUT GPIO.
    // if (!nmiOutConfig.lineName.empty())
    // {
    //     setGPIOOutput(nmiOutConfig.lineName, !nmiOutConfig.polarity,
    //                   nmiOutLine);
    // }

    // // Initialize POWER_OUT and RESET_OUT GPIO.
    // gpiod::line line;
    // if (!powerOutConfig.lineName.empty())
    // {
    //     if (!setGPIOOutput(powerOutConfig.lineName, !powerOutConfig.polarity,
    //                        line))
    //     {
    //         return -1;
    //     }
    // }
    // else
    // {
    //     lg2::error("powerOut name should be configured from json config file");
    //     return -1;
    // }

    // if (!resetOutConfig.lineName.empty())
    // {
    //     if (!setGPIOOutput(resetOutConfig.lineName, !resetOutConfig.polarity,
    //                        line))
    //     {
    //         return -1;
    //     }
    // }
    // else
    // {
    //     lg2::error("ResetOut name should be configured from json config file");
    //     return -1;
    // }
    // Release line
    // line.reset();

    // Initialize the power state and operating system state
    // powerState = PowerState::off;
    // operatingSystemState = OperatingSystemStateStage::Inactive;
    // Check power good

    // if (powerOkConfig.type == ConfigType::GPIO)
    // {
    //     if (psPowerOKLine.get_value() > 0 ||
    //         (sioEnabled &&
    //          (sioPowerGoodLine.get_value() == sioPwrGoodConfig.polarity)))
    //     {
    //         powerState = PowerState::on;
    //     }
    // }
    // else
    // {
    //     if (getProperty(powerOkConfig))
    //     {
    //         powerState = PowerState::on;
    //     }
    // }
    // Check if we need to start the Power Restore policy
    if (powerState != PowerState::on)
    {
        powerRestore.run();
    }

    if (nmiOutLine)
        nmiSourcePropertyMonitor();

    lg2::info("Initializing power state.");
    logStateTransition(powerState);

    // Power Control Service
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);

    // Power Control Interface
    hostIface =
        hostServer.add_interface("/xyz/openbmc_project/state/host" + node,
                                 "xyz.openbmc_project.State.Host");
    // Interface for IPMI/Redfish initiated host state transitions
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::gracefulPowerOffRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerOnRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerCycleRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot")
            {
                // if reset button is masked, ignore this
                if (!resetButtonMask)
                {
                    sendPowerControlEvent(Event::gracefulPowerCycleRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Reset Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                // if reset button is masked, ignore this
                if (!resetButtonMask)
                {
                    sendPowerControlEvent(Event::resetRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Reset Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else
            {
                lg2::error("Unrecognized host state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    hostIface->register_property("CurrentHostState",
                                 std::string(getHostState(powerState)));

    hostIface->initialize();

    lg2::info("DEBUG:: Created the host interface successfully");

    // Chassis Control Service
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(conn);

    // Chassis Control Interface
    chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis" + node,
                                    "xyz.openbmc_project.State.Chassis");

    chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerOffRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerOnRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerCycleRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else
            {
                lg2::error("Unrecognized chassis state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisIface->register_property("CurrentPowerState",
                                    std::string(getChassisState(powerState)));
    chassisIface->register_property("LastStateChangeTime", getCurrentTimeMs());

    chassisIface->initialize();

    lg2::info("DEBUG:: Created the chassis interface successfully");

#ifdef CHASSIS_SYSTEM_RESET
    // Chassis System Service
    sdbusplus::asio::object_server chassisSysServer =
        sdbusplus::asio::object_server(conn);

    // Chassis System Interface
    chassisSysIface = chassisSysServer.add_interface(
        "/xyz/openbmc_project/state/chassis_system0",
        "xyz.openbmc_project.State.Chassis");

    chassisSysIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                systemReset();
                addRestartCause(RestartCause::command);
            }
            else
            {
                lg2::error(
                    "Unrecognized chassis system state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisSysIface->register_property(
        "CurrentPowerState", std::string(getChassisState(powerState)));
    chassisSysIface->register_property("LastStateChangeTime",
                                       getCurrentTimeMs());

    chassisSysIface->initialize();

    if (!slotPowerConfig.lineName.empty())
    {
        if (!setGPIOOutput(slotPowerConfig.lineName, 1, slotPowerLine))
        {
            return -1;
        }

        slotPowerState = SlotPowerState::off;
        if (slotPowerLine.get_value() > 0)
        {
            slotPowerState = SlotPowerState::on;
        }

        chassisSlotIface = chassisSysServer.add_interface(
            "/xyz/openbmc_project/state/chassis_system" + node,
            "xyz.openbmc_project.State.Chassis");
        chassisSlotIface->register_property(
            "RequestedPowerTransition",
            std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
            [](const std::string& requested, std::string& resp) {
                if (requested ==
                    "xyz.openbmc_project.State.Chassis.Transition.On")
                {
                    slotPowerOn();
                }
                else if (requested ==
                         "xyz.openbmc_project.State.Chassis.Transition.Off")
                {
                    slotPowerOff();
                }
                else if (
                    requested ==
                    "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
                {
                    slotPowerCycle();
                }
                else
                {
                    lg2::error(
                        "Unrecognized chassis system state transition request.\n");
                    throw std::invalid_argument(
                        "Unrecognized Transition Request");
                    return 0;
                }
                resp = requested;
                return 1;
            });
        chassisSlotIface->register_property(
            "CurrentPowerState", std::string(getSlotState(slotPowerState)));
        chassisSlotIface->register_property("LastStateChangeTime",
                                            getCurrentTimeMs());
        chassisSlotIface->initialize();
    }
#endif
    // Buttons Service
    sdbusplus::asio::object_server buttonsServer =
        sdbusplus::asio::object_server(conn);

    if (!powerButtonConfig.lineName.empty())
    {
        // Power Button Interface
        power_control::powerButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/power",
            "xyz.openbmc_project.Chassis.Buttons");

        powerButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (requested)
                {
                    if (powerButtonMask)
                    {
                        return 1;
                    }
                    if (!setGPIOOutput(powerOutConfig.lineName,
                                       !powerOutConfig.polarity,
                                       powerButtonMask))
                    {
                        throw std::runtime_error("Failed to request GPIO");
                        return 0;
                    }
                    lg2::info("Power Button Masked.");
                }
                else
                {
                    if (!powerButtonMask)
                    {
                        return 1;
                    }
                    lg2::info("Power Button Un-masked");
                    powerButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check power button state
        bool powerButtonPressed;
        if (powerButtonConfig.type == ConfigType::GPIO)
        {
            powerButtonPressed = powerButtonLine.get_value() == 0;
        }
        else
        {
            powerButtonPressed = getProperty(powerButtonConfig) == 0;
        }

        powerButtonIface->register_property("ButtonPressed",
                                            powerButtonPressed);

        powerButtonIface->initialize();
    }

    if (!resetButtonConfig.lineName.empty())
    {
        // Reset Button Interface

        resetButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/reset",
            "xyz.openbmc_project.Chassis.Buttons");

        resetButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (requested)
                {
                    if (resetButtonMask)
                    {
                        return 1;
                    }
                    if (!setGPIOOutput(resetOutConfig.lineName,
                                       !resetOutConfig.polarity,
                                       resetButtonMask))
                    {
                        throw std::runtime_error("Failed to request GPIO");
                        return 0;
                    }
                    lg2::info("Reset Button Masked.");
                }
                else
                {
                    if (!resetButtonMask)
                    {
                        return 1;
                    }
                    lg2::info("Reset Button Un-masked");
                    resetButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check reset button state
        bool resetButtonPressed;
        if (resetButtonConfig.type == ConfigType::GPIO)
        {
            resetButtonPressed = resetButtonLine.get_value() == 0;
        }
        else
        {
            resetButtonPressed = getProperty(resetButtonConfig) == 0;
        }

        resetButtonIface->register_property("ButtonPressed",
                                            resetButtonPressed);

        resetButtonIface->initialize();
    }

    if (nmiButtonLine)
    {
        // NMI Button Interface
        nmiButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/nmi",
            "xyz.openbmc_project.Chassis.Buttons");

        nmiButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (nmiButtonMasked == requested)
                {
                    // NMI button mask is already set as requested, so no change
                    return 1;
                }
                if (requested)
                {
                    lg2::info("NMI Button Masked.");
                    nmiButtonMasked = true;
                }
                else
                {
                    lg2::info("NMI Button Un-masked.");
                    nmiButtonMasked = false;
                }
                // Update the mask setting
                current = nmiButtonMasked;
                return 1;
            });

        // Check NMI button state
        bool nmiButtonPressed;
        if (nmiButtonConfig.type == ConfigType::GPIO)
        {
            nmiButtonPressed = nmiButtonLine.get_value() == 0;
        }
        else
        {
            nmiButtonPressed = getProperty(nmiButtonConfig) == 0;
        }

        nmiButtonIface->register_property("ButtonPressed", nmiButtonPressed);

        nmiButtonIface->initialize();
    }

    if (nmiOutLine)
    {
        // NMI out Service
        sdbusplus::asio::object_server nmiOutServer =
            sdbusplus::asio::object_server(conn);

        // NMI out Interface
        nmiOutIface = nmiOutServer.add_interface(
            "/xyz/openbmc_project/control/host" + node + "/nmi",
            "xyz.openbmc_project.Control.Host.NMI");
        nmiOutIface->register_method("NMI", nmiReset);
        nmiOutIface->initialize();
    }

    if (idButtonLine)
    {
        // ID Button Interface
        idButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/id",
            "xyz.openbmc_project.Chassis.Buttons");

        // Check ID button state
        bool idButtonPressed;
        if (idButtonConfig.type == ConfigType::GPIO)
        {
            idButtonPressed = idButtonLine.get_value() == 0;
        }
        else
        {
            idButtonPressed = getProperty(idButtonConfig) == 0;
        }

        idButtonIface->register_property("ButtonPressed", idButtonPressed);

        idButtonIface->initialize();
    }

    // OS State Service
    sdbusplus::asio::object_server osServer =
        sdbusplus::asio::object_server(conn);

    // OS State Interface
    osIface = osServer.add_interface(
        "/xyz/openbmc_project/state/host" + node,
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Get the initial OS state based on POST complete
    //      Asserted, OS state is "Standby" (ready to boot)
    //      De-Asserted, OS state is "Inactive"
    lg2::info("DEBUG:: Before getting the initial OS state");

    // TODO: Replace with a NVL144/C2 CPU Boot indicator GPIO
    // OperatingSystemStateStage osState;
    // if (postCompleteConfig.type == ConfigType::GPIO)
    // {
    //     osState = postCompleteLine.get_value() == postCompleteConfig.polarity
    //                   ? OperatingSystemStateStage::Standby
    //                   : OperatingSystemStateStage::Inactive;
    // }
    // else
    // {
    //     osState = getProperty(postCompleteConfig) > 0
    //                   ? OperatingSystemStateStage::Standby
    //                   : OperatingSystemStateStage::Inactive;
    // }

    osIface->register_property(
        "OperatingSystemState",
        std::string(getOperatingSystemStateStage(operatingSystemState)));

    osIface->initialize();

    lg2::info("DEBUG:: After getting the initial OS state and registering the property");

    // Restart Cause Service
    sdbusplus::asio::object_server restartCauseServer =
        sdbusplus::asio::object_server(conn);

    // Restart Cause Interface
    restartCauseIface = restartCauseServer.add_interface(
        "/xyz/openbmc_project/control/host" + node + "/restart_cause",
        "xyz.openbmc_project.Control.Host.RestartCause");

    restartCauseIface->register_property(
        "RestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"));

    restartCauseIface->register_property(
        "RequestedRestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer")
            {
                addRestartCause(RestartCause::watchdog);
            }
            else
            {
                throw std::invalid_argument(
                    "Unrecognized RestartCause Request");
                return 0;
            }

            lg2::info("RestartCause requested: {RESTART_CAUSE}",
                      "RESTART_CAUSE", requested);
            resp = requested;
            return 1;
        });

    restartCauseIface->initialize();

    currentHostStateMonitor();

    if (!hpmStbyEnConfig.lineName.empty())
    {
        // Set to indicate BMC's power control module is ready to take
        // the inputs [PWR_GOOD] from the HPM FPGA
        gpiod::line hpmLine;
        if (!setGPIOOutput(hpmStbyEnConfig.lineName, hpmStbyEnConfig.polarity,
                           hpmLine))
        {
            return -1;
        }
    }

    io.run();

    return 0;
}
