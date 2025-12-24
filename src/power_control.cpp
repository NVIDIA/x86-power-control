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
#include "platform/nvl144/nvl144_power_control.hpp"

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


#ifdef USE_ACBOOT
static constexpr const char* powerACBootObject =
    "/xyz/openbmc_project/control/host0/ac_boot";
static constexpr const char* powerACBootIface =
    "xyz.openbmc_project.Common.ACBoot";
#endif // USE_ACBOOT

namespace match_rules = sdbusplus::bus::match::rules;

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

// Unique POH TImer to base class
static void pohCounterTimerStart(std::shared_ptr<sdbusplus::asio::connection> conn, const PowerControl& powerControl)
{
    lg2::info("POH timer started");
    // Set the time-out as 1 hour, to align with POH command in ipmid
    pohCounterTimer.expires_after(std::chrono::hours(1));
    pohCounterTimer.async_wait([conn, &powerControl](const boost::system::error_code& ec) {
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

        if (powerControl.getHostState() !=
            "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return;
        }

        conn->async_method_call(
            [conn](boost::system::error_code ec,
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

        pohCounterTimerStart(conn, powerControl);
    });
}

static void currentHostStateMonitor(std::shared_ptr<sdbusplus::asio::connection> conn, const PowerControl& powerControl)
{
    if (powerControl.getHostState() ==
        "xyz.openbmc_project.State.Host.HostState.Running")
    {
        pohCounterTimerStart(conn, powerControl);
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
        [conn, &powerControl](sdbusplus::message_t& message) {
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
                pohCounterTimerStart(conn, powerControl);
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




static void sioOnControlHandler(bool state)
{
    lg2::info("SIO_ONCONTROL value changed: {VALUE}", "VALUE",
              static_cast<int>(state));
}

#ifdef CHASSIS_SYSTEM_RESET
static constexpr auto systemdBusname = "org.freedesktop.systemd1";
static constexpr auto systemdPath = "/org/freedesktop/systemd1";
static constexpr auto systemdInterface = "org.freedesktop.systemd1.Manager";
static constexpr auto systemTargetName = "chassis-system-reset.target";

void systemReset(std::shared_ptr<sdbusplus::asio::connection> conn)
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

} // namespace power_control

int main(int argc, char* argv[])
{
    using namespace power_control;
    static boost::asio::io_context io;
    PersistentState appState;

    static std::string node = "0";
    static const std::string appName = "power-control";

    if (argc > 1)
    {
        node = argv[1];
    }
    lg2::info("Start Chassis power control service for host : {NODE}", "NODE",
              node);
    
    std::shared_ptr<sdbusplus::asio::connection> conn = std::make_shared<sdbusplus::asio::connection>(io);
    NVL144PowerControl powerControl(io, conn, "config/power-config-host0.json", node, appState);
    PowerRestoreController powerRestore(io, conn, node, powerControl, appState);

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

    // NMI source property monitor is initialized by the powerControl object
    // if NMIOut is configured in powerSignalMap
    powerControl.nmiSourcePropertyMonitor();

    lg2::info("Initializing power state.");

    // D-Bus interfaces (host, chassis, boot progress, buttons, OS state, restart cause)
    // are now initialized by the PowerControl base class constructor

    currentHostStateMonitor(conn);

    return 0;
}
