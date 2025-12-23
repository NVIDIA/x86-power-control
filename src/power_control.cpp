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
#include "power_control_base.hpp"
#include "power_restore.hpp"

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
#include <chrono>

// For input event monitoring - PDB IOX interrupt lines are not connected to BMC/SMM
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cerrno>
#include <cstring>

//  For starting cpu boot services
#include <sdbusplus/bus.hpp>

namespace power_control
{

// Event enum is now defined inside PowerControl class in power_control_base.hpp
using Event = PowerControl::Event;

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

// Helper function to set boot progress
static void setBootProgress(const std::string& bootProgressStage)
{
    if (bootProgressIface)
    {
        bootProgressIface->set_property("BootProgress", bootProgressStage);
        
        // Update timestamp
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
        bootProgressIface->set_property("BootProgressLastUpdate", 
                                       static_cast<uint64_t>(timestamp));
        
        lg2::info("Boot progress updated to: {PROGRESS}", "PROGRESS", bootProgressStage);
    }
}

// Helper function to set OEM boot progress
static void setBootProgressOem(const std::string& oemProgress)
{
    if (bootProgressIface)
    {
        bootProgressIface->set_property("BootProgressOem", oemProgress);
        lg2::info("Boot progress OEM updated to: {OEM_PROGRESS}", "OEM_PROGRESS", oemProgress);
    }
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


// Helper function to find the correct input event device by name
static std::string findInputEventDevice(const std::string& deviceName)
{
    // Search through /dev/input/eventX devices to find the one matching our name
    for (int i = 0; i < 32; i++)
    {
        std::string eventPath = "/dev/input/event" + std::to_string(i);
        std::string namePath = "/sys/class/input/event" + std::to_string(i) + "/device/name";
        
        std::ifstream nameFile(namePath);
        if (nameFile.is_open())
        {
            std::string name;
            std::getline(nameFile, name);
            if (name == deviceName)
            {
                lg2::info("Found input device {DEVICE_NAME} at {EVENT_PATH}",
                         "DEVICE_NAME", deviceName, "EVENT_PATH", eventPath);
                return eventPath;
            }
        }
    }
    return "";
}

// Helper function to monitor Linux input events (for gpio_keys_polled driver)
static void waitForInputEvent(
    const std::string& name, const std::function<void(bool)>& eventHandler,
    uint16_t keyCode, boost::asio::posix::stream_descriptor& event,
    int* stateTracker = nullptr)
{
    event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [name, eventHandler, keyCode, &event, stateTracker](const boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("{INPUT_NAME} fd handler error: {ERROR_MSG}",
                           "INPUT_NAME", name, "ERROR_MSG", ec.message());
                return;
            }
            
            struct input_event inputEvent;
            ssize_t bytesRead = read(event.native_handle(), &inputEvent, sizeof(inputEvent));
            
            if (bytesRead != sizeof(inputEvent))
            {
                if (bytesRead < 0)
                {
                    lg2::error("{INPUT_NAME} read error: {ERROR}",
                               "INPUT_NAME", name, "ERROR", strerror(errno));
                }
                else
                {
                    lg2::error("{INPUT_NAME} read error: incomplete event (got {BYTES} bytes)",
                               "INPUT_NAME", name, "BYTES", bytesRead);
                }
                waitForInputEvent(name, eventHandler, keyCode, event, stateTracker);
                return;
            }
            
            // We only care about EV_KEY events with our specific key code
            if (inputEvent.type == EV_KEY && inputEvent.code == keyCode)
            {
                lg2::info("{INPUT_NAME} event: code={KEY_CODE:#x} value={VALUE}",
                         "INPUT_NAME", name, "KEY_CODE", inputEvent.code, 
                         "VALUE", inputEvent.value);
                
                // Update state tracker if provided
                if (stateTracker != nullptr)
                {
                    *stateTracker = inputEvent.value;
                }
                
                // Value 1 = pressed (high), 0 = released (low)
                eventHandler(inputEvent.value == 1);
            }
            
            // Continue waiting for next event
            waitForInputEvent(name, eventHandler, keyCode, event, stateTracker);
        });
}

// Request monitoring of Linux input events (for gpio_keys_polled driver)
static bool requestInputEvents(
    const std::string& deviceName, const std::string& signalName,
    uint16_t keyCode, const std::function<void(bool)>& handler,
    boost::asio::posix::stream_descriptor& eventDescriptor,
    int* stateTracker = nullptr)
{
    // Find the input device
    std::string eventPath = findInputEventDevice(deviceName);
    if (eventPath.empty())
    {
        lg2::error("Failed to find input device {DEVICE_NAME}",
                   "DEVICE_NAME", deviceName);
        return false;
    }
    
    // Open the event device
    int fd = open(eventPath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        lg2::error("Failed to open {EVENT_PATH}: {ERROR}",
                   "EVENT_PATH", eventPath, "ERROR", strerror(errno));
        return false;
    }
    
    // Assign to the stream descriptor
    eventDescriptor.assign(fd);
    
    // Start waiting for events
    waitForInputEvent(signalName, handler, keyCode, eventDescriptor, stateTracker);
    
    lg2::info("Successfully set up input event monitoring for {SIGNAL_NAME} on {EVENT_PATH} with key code {KEY_CODE:#x}",
             "SIGNAL_NAME", signalName, "EVENT_PATH", eventPath, "KEY_CODE", keyCode);
    
    return true;
}

// Helper function to read GB300 PDB Main Power OK state from cached value
// Since gpio_keys_polled has exclusive control, we can't request the GPIO directly
// Instead, we track the state based on input events
static int getGB300PdbMainPowerOkValue()
{
    return gb300pdbMainPowerOkState;
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
    else if(powerContext.presence.gb300_pdb)
    {
        timeout = TimerMap["GB300PdbMainPowerOkWatchdogMs"];
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

    detectBoardPresence();
    // Check if no boards or no PDB present - error condition
    if (!powerContext.presence.board0)
    {
        lg2::error("No Board 0 present. Rejecting power request.");
        return;  // Return without taking action
    }
    else if (!powerContext.presence.nvl144_pdb && 
             !powerContext.presence.c2_pdb && 
             !powerContext.presence.gb300_pdb)
    {
        lg2::error("No PDB present. Rejecting power request.");
        return;
    }

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

            // Check if all present boards already have their power rails off
            // For each board: if not present OR (present AND power is de-asserted)
            // This ensures we only check get_value() on boards that are present
            if ((!powerContext.presence.nvl144_pdb || (nvl144pdbMainPowerOkLine.get_value() == !nvl144pdbMainPowerOkConfig.polarity)) &&
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
                        lg2::info("NVL144 PDB Main Power OK is already asserted. Ensuring PDB Main Power Enable is asserted. Commencing HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, asserting E1S Power Enable, de-asserting BMC SDD Reset, and asserting Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");

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
                        // Assert E1S Power Enable
                        if(e1sPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(e1sPowerEnableConfig.lineName, e1sPowerEnableConfig.polarity, e1sPowerEnableLine);
                        }
                        // De-assert BMC SDD Reset
                        if(bmcSSDResetConfig.lineName.empty())
                        {
                            setGPIOOutput(bmcSSDResetConfig.lineName, !bmcSSDResetConfig.polarity, bmcSSDResetLine);
                        }
                        // Assert SMM USB Power Enable
                        if(usbPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(usbPowerEnableConfig.lineName, usbPowerEnableConfig.polarity, usbPowerEnableLine);
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
                // C2 Non-Intelligent PDB Sequence Start
                else if(powerContext.presence.c2_pdb && !powerContext.presence.c2_pdb_mcu)
                {
                    // C2 PDB Main Power OK is already asserted
                    if(c2pdbPSUPowerOkLine.get_value() == c2pdbPSUPowerOkConfig.polarity)
                    {
                        // Enable C2 PDB 12V Rails
                        lg2::info("C2 PDB Main Power OK is already asserted. Ensuring C2 PDB Main Power Enable is asserted. Asserting 12V PSU Enable Lines.");
                        setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
                        setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
                        setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);

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
                        // Assert SMM USB Power Enable
                        if(usbPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(usbPowerEnableConfig.lineName, usbPowerEnableConfig.polarity, usbPowerEnableLine);
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

                        // Assert C2 PDB PSU Power Enable
                        setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);

                        // start the C2 PDB Main Power Ok Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                        pdbMainPowerOkWatchdogTimerStart();
                        setPowerState(PowerState::waitForPDBMainPowerOk);
                    }
                    
                }
                else if (powerContext.presence.c2_pdb && powerContext.presence.c2_pdb_mcu)
                {
                    // C2 PDB Main Power OK is already asserted
                    if(c2pdbPSUPowerOkLine.get_value() == c2pdbPSUPowerOkConfig.polarity)
                    {
                        
                    }
                }
                // No PDB present - HPM Board sequence Start
                else if(powerContext.presence.gb300_pdb)
                {

                    // NVL144 PDB Main Power OK is already asserted
                    // if (gb300pdbMainPowerOkLine.get_value() == gb300pdbMainPowerOkConfig.polarity)
                    // GB300 PDB Main Power OK is already asserted
                    int gb300PwrOkValue = getGB300PdbMainPowerOkValue();
                    if (gb300PwrOkValue >= 0 && gb300PwrOkValue == gb300pdbMainPowerOkConfig.polarity)
                    {
                        lg2::info("GB300 PDB Main Power OK is already asserted. Ensuring PDB Main Power Enable is asserted. Commencing HPM Board Power Sequencing.");

                        if (!gb300pdbMainPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(gb300pdbMainPowerEnableConfig.lineName, gb300pdbMainPowerEnableConfig.polarity, gb300pdbMainPowerEnableLine);
                        }
                        // Begin HPM Sequencing. Assert Board 0 and/or Board 1 Pre System Reset
                        if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                        {
                            setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                        }
                        if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                        {
                            setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                        }

                        // Assert SMM USB Power Enable
                        if(usbPowerEnableConfig.lineName.empty())
                        {
                            setGPIOOutput(usbPowerEnableConfig.lineName, usbPowerEnableConfig.polarity, usbPowerEnableLine);
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
                    else
                    {
                        // Assert GB300 PDB Main Power Enable
                        setGPIOOutput(gb300pdbMainPowerEnableConfig.lineName, gb300pdbMainPowerEnableConfig.polarity, gb300pdbMainPowerEnableLine);

                        // start the GB300 PDB Main Power Ok Watchdog Timer (uses timeout configured from config/power-config-host0.json)
                        pdbMainPowerOkWatchdogTimerStart();
                        setPowerState(PowerState::waitForPDBMainPowerOk);
                    }
                }
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
                lg2::info("NVL144 PDB Main Power OK Asserted. Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, asserting E1S Power Enable, de-asserting BMC SDD Reset, and asserting Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");
                
                // Assert Board 0 and/or Board 1 Pre System Reset
                if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                }
                if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                }

                // Assert E1S Power Enable
                if(e1sPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(e1sPowerEnableConfig.lineName, e1sPowerEnableConfig.polarity, e1sPowerEnableLine);
                }

                // De-assert BMC SDD Reset
                if(bmcSSDResetConfig.lineName.empty())
                {
                    setGPIOOutput(bmcSSDResetConfig.lineName, !bmcSSDResetConfig.polarity, bmcSSDResetLine);
                }

                if(usbPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(usbPowerEnableConfig.lineName, usbPowerEnableConfig.polarity, usbPowerEnableLine);
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
                lg2::info("C2 PDB PSU Power OK Asserted. Asserting C2 PDB 12V Rails and Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, USB Power Enable, and HPM Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");

                setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
                setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
                setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);

                // Assert Board 0 and/or Board 1 Pre System Reset
                if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                }
                if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                {
                    setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                }

                // Assert USB Power Enable
                if(usbPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(usbPowerEnableConfig.lineName, usbPowerEnableConfig.polarity, usbPowerEnableLine);
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
        case Event::gb300pdbMainPowerOkAssert:
            if(powerContext.action == PowerAction::POWER_ON && powerContext.presence.gb300_pdb)
            {
                pdbMainPowerOkWatchdogTimer.cancel(); // Cancel the PDB Main Power OK watchdog timer
                lg2::info("GB300 PDB Main Power OK Asserted. Conducting HPM Board Power Sequencing. Asserting HPM Board Pre System Reset, USB Power Enable, and HPM Run Power Enable Lines. Waiting For HPM Board Power Good Assertion Event...");
                

                 // Assert Board 0 and/or Board 1 Pre System Reset
                 if(powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
                 {
                     setGPIOOutput(board0PreSystemResetConfig.lineName, board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
                 }
                 if(powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
                 {
                     setGPIOOutput(board1PreSystemResetConfig.lineName, board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
                 }

                 // Assert USB Power Enable
                 if(!usbPowerEnableConfig.lineName.empty())
                 {
                     setGPIOOutput(usbPowerEnableConfig.lineName, usbPowerEnableConfig.polarity, usbPowerEnableLine);
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
                else if(powerContext.presence.gb300_pdb)
                {
                    setGPIOOutput(gb300pdbMainPowerEnableConfig.lineName, !gb300pdbMainPowerEnableConfig.polarity, gb300pdbMainPowerEnableLine);
                }
            }
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

// Helper function: De-assert board run power enable lines (used for power-on fault cleanup)
static void deassertBoardRunPowerEnables()
{
    if (powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
    {
        setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
    }
    
    if (powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
    {
        setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
    }
}

// Helper function: De-assert board pre-system reset lines (common cleanup operation)
static void deassertBoardPreSystemResets()
{
    if (powerContext.presence.board0 && !board0PreSystemResetConfig.lineName.empty())
    {
        setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
    }
    
    if (powerContext.presence.board1 && !board1PreSystemResetConfig.lineName.empty())
    {
        setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
    }
}

// Helper function: Complete power-on fault cleanup sequence
static void completePowerOnFaultCleanup()
{
    lg2::info("Conducting HPM Main Power On Fault cleanup: De-asserting Pre System Reset & Run Power Enable. Host Main Power On sequence failed! Setting Host Power State to Off.");
    deassertBoardPreSystemResets();
    deassertBoardRunPowerEnables();
    setPowerState(PowerState::off);
    powerContext.action = PowerAction::NONE;
}

// Helper function: Complete force-off cleanup sequence
// Forward declaration for functions defined later
static void forceOffCleanUpSequence();
static void completeForcefulShutdownSequence();

static void completeForceOffCleanup()
{
    lg2::info("Conducting Force Off Cleanup Sequence: De-asserting Pre System Reset & CPU Shutdown Force Lines. Setting Host Power State to Off.");
    completeForcefulShutdownSequence(); // Reuses existing helper that de-asserts board control GPIOs and transitions to off
}

// Helper function: Get PDB name string for logging
static const char* getPDBName()
{
    if (powerContext.presence.nvl144_pdb) return "NVL144 PDB";
    if (powerContext.presence.gb300_pdb) return "GB300 PDB";
    if (powerContext.presence.c2_pdb) return "C2 PDB";
    return "PDB";
}

// Refactored function with consolidated logic
static void powerStateWaitForPDBMainPowerOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    
    // Handle PDB power OK de-assertion events (success cases)
    if (event == Event::nvl144pdbMainPowerOkDeAssert || 
        event == Event::gb300pdbMainPowerOkDeAssert || 
        event == Event::c2pdbPSUPowerOkDeAssert)
    {
        pdbMainPowerOkWatchdogTimer.cancel();
        
        if (powerContext.action == PowerAction::POWER_ON)
        {
            lg2::info("{PDB_NAME} PDB Main Power Rail powered down successfully.", "PDB_NAME", getPDBName());
            completePowerOnFaultCleanup();
        }
        else if (powerContext.action == PowerAction::FORCE_OFF)
        {
            lg2::info("{PDB_NAME} PDB Main Power Rail powered down. Host Forceful Shutdown sequence completed successfully!", "PDB_NAME", getPDBName());
            completeForceOffCleanup();
        }
        return;
    }
    
    // Handle watchdog timer expiration (timeout/failure cases)
    if (event == Event::pdbMainPowerOkWatchdogTimerExpired)
    {
            if (powerContext.action == PowerAction::POWER_ON)
            {
                lg2::error("Failed to Power Down {PDB_NAME} Main Power Rail. PDB & HPM power domain inconsistency.", "PDB_NAME", getPDBName());
                completePowerOnFaultCleanup();
            }
            else if (powerContext.action == PowerAction::FORCE_OFF)
            {
                lg2::error("Failed to Power Down {PDB_NAME} Main Power Rail. Host Forceful Shutdown sequence failed!", "PDB_NAME", getPDBName());
                completeForceOffCleanup();
            }
        return;
    }
    
    // Default case
    lg2::info("No action taken.");
}

// Helper function: Check if all required boards are powered on
static bool areAllRequiredBoardsPoweredOn()
{
    bool board0On = !powerContext.presence.board0 || 
                    (board0RunPowerPGLine.get_value() == board0RunPowerPGConfig.polarity);
    bool board1On = !powerContext.presence.board1 || 
                    (board1RunPowerPGLine.get_value() == board1RunPowerPGConfig.polarity);
    return board0On && board1On;
}

// Helper function: Transition to CPU reset de-assert wait state
static void transitionToCPUResetWait()
{
    hpmPowerGoodWatchdogTimer.cancel();
    
    // Log appropriate message based on board configuration
    if (powerContext.presence.board0 && powerContext.presence.board1)
    {
        lg2::info("HPM Board 0 & Board 1 Run Power Good Asserted. De-asserting Pre System Reset Lines. Waiting for CPU Reset De-assertion...");
    }
    else if (powerContext.presence.board0)
    {
        lg2::info("HPM Board 0 Run Power Good Asserted. De-asserting Pre System Reset Lines. Waiting for CPU Reset De-assertion...");
    }
    
    deassertBoardPreSystemResets();
    cpuResetWatchdogTimerStart();
    setPowerState(PowerState::waitForCPUResetDeAssert);
}

// Helper function: Power down NVL144 PDB for power-on fault recovery
static void powerDownNVL144PDBForFaultRecovery()
{
    lg2::info("Powering down {PDB_NAME} Main Power Rail to establish HPM/PDB consistency. De-asserting Main Power Enable.", "PDB_NAME", getPDBName());
    setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
    pdbMainPowerOkWatchdogTimerStart();
    setPowerState(PowerState::waitForPDBMainPowerOff);
}

// Helper function: Power down GB300 PDB for power-on fault recovery
static void powerDownGB300PDBForFaultRecovery()
{
    lg2::info("Powering down {PDB_NAME} Main Power Rail to establish HPM/PDB consistency. De-asserting Main Power Enable.", "PDB_NAME", getPDBName());
    setGPIOOutput(gb300pdbMainPowerEnableConfig.lineName, !gb300pdbMainPowerEnableConfig.polarity, gb300pdbMainPowerEnableLine);
    pdbMainPowerOkWatchdogTimerStart();
    setPowerState(PowerState::waitForPDBMainPowerOff);
}

// Helper function: Power down C2 PDB for power-on fault recovery
static void powerDownC2PDBForFaultRecovery()
{
    lg2::info("Powering down {PDB_NAME} Main Power Rail to establish HPM/PDB consistency. De-asserting 12V Rails and PSU Power Enable.", "PDB_NAME", getPDBName());
    setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, !c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
    setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, !c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
    setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, !c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);
    setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, !c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);
    pdbMainPowerOkWatchdogTimerStart();
    setPowerState(PowerState::waitForPDBMainPowerOff);
}

// Helper function: Handle power-on fault by powering down PDB
static void handlePowerOnFault()
{
    lg2::error("HPM Main Power On Fault detected. Main Power On sequence failed. Conducting HPM/{PDB_NAME} reconciliation sequence.", "PDB_NAME", getPDBName());
    
    if (powerContext.presence.nvl144_pdb)
    {
        powerDownNVL144PDBForFaultRecovery();
    }
    else if (powerContext.presence.gb300_pdb)
    {
        powerDownGB300PDBForFaultRecovery();
    }
    else if (powerContext.presence.c2_pdb)
    {
        powerDownC2PDBForFaultRecovery();
    }
}

// Refactored function with consolidated logic
static void powerStateWaitForHPMPowerGoodAssert(const Event event)
{
    logEvent(__FUNCTION__, event);
    
    // Only process during POWER_ON action
    if (powerContext.action != PowerAction::POWER_ON)
    {
        lg2::info("No action taken.");
        return;
    }
    
    // Handle board power good assertion events
    if (event == Event::board0RunPowerPGAssert || event == Event::board1RunPowerPGAssert)
    {
        // Check if all required boards are now powered on
        if (areAllRequiredBoardsPoweredOn())
        {
            transitionToCPUResetWait();
        }
        else
        {
            // Still waiting for other board(s)
            if (event == Event::board0RunPowerPGAssert)
            {
                lg2::info("HPM Board 0 Run Power Good Asserted. Waiting for Board 1 Run Power Good Assertion...");
            }
            else
            {
                lg2::info("HPM Board 1 Run Power Good Asserted. Waiting for Board 0 Run Power Good Assertion...");
            }
        }
        return;
    }
    
    // Handle watchdog timer expiration
    if (event == Event::hpmPowerGoodWatchdogTimerExpired)
    {
        handlePowerOnFault();
        return;
    }
    
    // Default case
    lg2::info("No action taken.");
}


// Helper function: Check if all boards are powered off
static bool areAllBoardsPoweredOff()
{
    bool board0Off = !powerContext.presence.board0 || 
                    (powerContext.presence.board0 && (board0RunPowerPGLine.get_value() == !board0RunPowerPGConfig.polarity));
    bool board1Off = !powerContext.presence.board1 || 
                    (powerContext.presence.board1 && (board1RunPowerPGLine.get_value() == !board1RunPowerPGConfig.polarity));
    return board0Off && board1Off;
}

// Helper function: De-assert board control GPIOs for force off cleanup
static void forceOffCleanUpSequence()
{
    if (powerContext.presence.board0)
    {
        if (!board0PreSystemResetConfig.lineName.empty())
            setGPIOOutput(board0PreSystemResetConfig.lineName, !board0PreSystemResetConfig.polarity, board0PreSystemResetLine);
        if (!board0CpuShutdownForceConfig.lineName.empty())
            setGPIOOutput(board0CpuShutdownForceConfig.lineName, !board0CpuShutdownForceConfig.polarity, board0CpuShutdownForceLine);
    }
    
    if (powerContext.presence.board1)
    {
        if (!board1PreSystemResetConfig.lineName.empty())
            setGPIOOutput(board1PreSystemResetConfig.lineName, !board1PreSystemResetConfig.polarity, board1PreSystemResetLine);
        if (!board1CpuShutdownForceConfig.lineName.empty())
            setGPIOOutput(board1CpuShutdownForceConfig.lineName, !board1CpuShutdownForceConfig.polarity, board1CpuShutdownForceLine);
    }
}

// Helper function: Complete last of forceful shutdown sequence
static void completeForcefulShutdownSequence()
{
    forceOffCleanUpSequence();
    setPowerState(PowerState::off);
    
    // Set boot progress to Unspecified when force off completes successfully
    setBootProgress("xyz.openbmc_project.State.Boot.Progress.ProgressStages.Unspecified");
    
    powerContext.action = PowerAction::NONE;
}

// Helper function: Handle NVL144 PDB power off sequence
static void handleNVL144PDBPowerOff()
{
    if (nvl144pdbMainPowerOkLine.get_value() == !nvl144pdbMainPowerOkConfig.polarity)
    {
        // Power OK already de-asserted - complete shutdown
        lg2::info("NVL144 PDB Main Power OK is de-asserted. Completing shutdown sequence.");
        setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
        completeForcefulShutdownSequence();
    }
    else
    {
        // Power OK still asserted - need to wait for it
        lg2::info("NVL144 PDB Main Power OK is asserted. De-asserting Main Power Enable and waiting for Power OK de-assertion.");
        setGPIOOutput(nvl144pdbMainPowerEnableConfig.lineName, !nvl144pdbMainPowerEnableConfig.polarity, nvl144pdbMainPowerEnableLine);
        pdbMainPowerOkWatchdogTimerStart();
        setPowerState(PowerState::waitForPDBMainPowerOff);
    }
}

// Helper function: Handle GB300 PDB power off sequence
static void handleGB300PDBPowerOff()
{
    // if (gb300pdbMainPowerOkLine.get_value() == !gb300pdbMainPowerOkConfig.polarity)
    int gb300PwrOkValue = getGB300PdbMainPowerOkValue();
    if (gb300PwrOkValue >= 0 && gb300PwrOkValue == !gb300pdbMainPowerOkConfig.polarity)
    {
        // Power OK already de-asserted - complete shutdown
        lg2::info("GB300 PDB Main Power OK is de-asserted. Completing shutdown sequence.");
        setGPIOOutput(gb300pdbMainPowerEnableConfig.lineName, !gb300pdbMainPowerEnableConfig.polarity, gb300pdbMainPowerEnableLine);
        completeForcefulShutdownSequence();
    }
    else
    {
        // Power OK still asserted - need to wait for it
        lg2::info("GB300 PDB Main Power OK is asserted. De-asserting Main Power Enable and waiting for Power OK de-assertion.");
        setGPIOOutput(gb300pdbMainPowerEnableConfig.lineName, !gb300pdbMainPowerEnableConfig.polarity, gb300pdbMainPowerEnableLine);
        pdbMainPowerOkWatchdogTimerStart();
        setPowerState(PowerState::waitForPDBMainPowerOff);
    }
}

// Helper function: Handle C2 PDB power off sequence
static void handleC2PDBPowerOff()
{
    if (c2pdbPSUPowerOkLine.get_value() == !c2pdbPSUPowerOkConfig.polarity)
    {
        // PSU Power OK already de-asserted - complete shutdown
        lg2::info("C2 PDB PSU Power OK is de-asserted. Completing shutdown sequence.");
        setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, !c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
        setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, !c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
        setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, !c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);
        setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, !c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);
        completeForcefulShutdownSequence();
    }
    else
    {
        // PSU Power OK still asserted - need to wait for it
        lg2::info("C2 PDB PSU Power OK is asserted. De-asserting 12V rails and PSU Power Enable, waiting for Power OK de-assertion.");
        setGPIOOutput(c2pdb_12V_HPMEnableConfig.lineName, !c2pdb_12V_HPMEnableConfig.polarity, c2pdb_12V_HPMEnableLine);
        setGPIOOutput(c2pdb_12V_GPU1EnableConfig.lineName, !c2pdb_12V_GPU1EnableConfig.polarity, c2pdb_12V_GPU1EnableLine);
        setGPIOOutput(c2pdb_12V_GPU2EnableConfig.lineName, !c2pdb_12V_GPU2EnableConfig.polarity, c2pdb_12V_GPU2EnableLine);
        setGPIOOutput(c2pdbPSUPowerEnableConfig.lineName, !c2pdbPSUPowerEnableConfig.polarity, c2pdbPSUPowerEnableLine);
        pdbMainPowerOkWatchdogTimerStart();
        setPowerState(PowerState::waitForPDBMainPowerOff);
    }
}

// Helper function: Conduct PDB power off sequence based on which PDB is present
static void conductPDBPowerOffSequence()
{
    if (powerContext.presence.nvl144_pdb)
    {
        handleNVL144PDBPowerOff();
    }
    else if (powerContext.presence.gb300_pdb)
    {
        handleGB300PDBPowerOff();
    }
    else if (powerContext.presence.c2_pdb)
    {
        handleC2PDBPowerOff();
    }
}

// Refactored main function
static void powerStateWaitForHPMPowerGoodDeAssert(const Event event)
{
    logEvent(__FUNCTION__, event);
    
    // Only handle board power good de-assert events during force-off
    if (powerContext.action != PowerAction::FORCE_OFF)
    {
        if (event == Event::hpmPowerGoodWatchdogTimerExpired)
        {
            lg2::error("HPM Power Good Watchdog Timer Expired. Host Forceful Shutdown sequence failed! Conducting Cleanup Sequence: De-asserting Pre System Reset & Shutdown Force lines. Setting Host Power State to On.");
            forceOffCleanUpSequence();
            setPowerState(PowerState::on);
            powerContext.action = PowerAction::NONE;
            return;
        }
        lg2::info("No action taken.");
        return;
    }

    switch (event)
    {
        case Event::board0RunPowerPGDeAssert:
        case Event::board1RunPowerPGDeAssert:
        {
            // Check if all boards are now powered off
            if (areAllBoardsPoweredOff())
            {
                hpmPowerGoodWatchdogTimer.cancel();
                lg2::info("All HPM boards powered off. Conducting PDB power off sequence.");
                conductPDBPowerOffSequence();
            }
            else
            {
                // Still waiting for other board(s)
                if (event == Event::board0RunPowerPGDeAssert)
                {
                    lg2::info("HPM Board 0 Run Power Good de-asserted. Waiting for Board 1 Run Power Good de-assertion...");
                }
                else
                {
                    lg2::info("HPM Board 1 Run Power Good de-asserted. Waiting for Board 0 Run Power Good de-assertion...");
                }
            }
            break;
        }
        
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
                // If NVL144: de-assert E1S Power Enable, BMC SSD Reset
                lg2::info("CPU Reset Indicator Asserted. CPUs are out of reset. De-asserting Run Power Enable Lines and USB Power Enable. Waiting for HPM Board Power Good Assertion Events...");

                if(powerContext.presence.board0 && !board0RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board0RunPowerEnableConfig.lineName, !board0RunPowerEnableConfig.polarity, board0RunPowerEnableLine);
                }
                if(powerContext.presence.board1 && !board1RunPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(board1RunPowerEnableConfig.lineName, !board1RunPowerEnableConfig.polarity, board1RunPowerEnableLine);
                }
                if(!usbPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(usbPowerEnableConfig.lineName, !usbPowerEnableConfig.polarity, usbPowerEnableLine);
                }

                // If NVL144: de-assert E1S Power Enable, BMC SSD Reset
                if(powerContext.presence.nvl144_pdb && !e1sPowerEnableConfig.lineName.empty())
                {
                    setGPIOOutput(e1sPowerEnableConfig.lineName, !e1sPowerEnableConfig.polarity, e1sPowerEnableLine);
                }
                if(powerContext.presence.nvl144_pdb && !bmcSSDResetConfig.lineName.empty())
                {
                    setGPIOOutput(bmcSSDResetConfig.lineName, bmcSSDResetConfig.polarity, bmcSSDResetLine);
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
                lg2::info("CPU Reset De-asserted. CPUs are out of reset. Host Power On Sequence completed Successfully! Setting Host Power State to On.");
                setPowerState(PowerState::on);
                powerContext.action = PowerAction::NONE;
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

void triggerCpuBootDone()
{
    try
    {
        auto bus = sdbusplus::bus::new_default();
        auto method = bus.new_method_call(
            "org.freedesktop.systemd1",
            "/org/freedesktop/systemd1",
            "org.freedesktop.systemd1.Manager",
            "StartUnit");
        
        method.append("cpu-boot-done.service", "replace");
        bus.call_noreply(method);
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to start cpu-boot-done.service: {ERROR_MSG}", "ERROR_MSG", e.what());
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

static void gb300pdbMainPowerOkHandler(bool state)
{
    Event powerControlEvent = (state == gb300pdbMainPowerOkConfig.polarity)
                                  ? Event::gb300pdbMainPowerOkAssert
                                  : Event::gb300pdbMainPowerOkDeAssert;
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
    static boost::asio::io_context io;
    PersistentState appState;
    PowerRestoreController powerRestore(io);

    static std::string node = "0";
    static const std::string appName = "power-control";\

    if (argc > 1)
    {
        node = argv[1];
    }
    lg2::info("Start Chassis power control service for host : {NODE}", "NODE",
              node);


    NVL144PowerControl powerControl(io, "config/power-config-host0.json", node);

#ifdef USE_PLT_RST
    sdbusplus::bus::match_t pltRstMatch(
        *conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='xyz.openbmc_project.State.Host.Misc'",
        hostMiscHandler);
#endif

    if (powerControl.getPowerStateName() != "on")
    {
        powerRestore.run();
    }

    if (nmiOutLine)
        nmiSourcePropertyMonitor();

    lg2::info("Initializing power state.");

    // D-Bus interfaces (host, chassis, boot progress, buttons, OS state, restart cause)
    // are now initialized by the PowerControl base class constructor

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
