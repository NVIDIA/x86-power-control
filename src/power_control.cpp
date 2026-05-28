// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "config.h"

#include "platform/c2/c2_power_control.hpp"
#include "platform/nvl72/nvl72_power_control.hpp"
#include "platform/vr_nvl8/vr_nvl8_power_control.hpp"
#include "power_control_base.hpp"
#include "power_restore.hpp"

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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <regex>
#include <string_view>

// For input event monitoring - PDB IOX interrupt lines are not connected to
// BMC/SMM
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

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
    auto it = TimerMap.find("SlotPowerCycleMs");
    if (it == TimerMap.end())
    {
        lg2::error("Timer config 'SlotPowerCycleMs' not found in TimerMap");
        return;
    }
    slotPowerCycleTimer.expires_after(std::chrono::milliseconds(it->second));
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

static std::string readPlatformType(const std::string& configPath)
{
    std::ifstream configFile(configPath);
    if (!configFile.is_open())
    {
        lg2::error("Cannot open config path '{PATH}'", "PATH", configPath);
        throw std::runtime_error("Failed to open config file: " + configPath);
    }

    auto jsonData = nlohmann::json::parse(configFile, nullptr, true, true);
    if (jsonData.contains("platform-type"))
    {
        return jsonData["platform-type"].get<std::string>();
    }

    return "nvl72";
}

static std::unique_ptr<PowerControl> createPowerControl(
    const std::string& platformType, boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configPath, const std::string& node,
    PersistentState& appState)
{
    lg2::info("Creating power control for platform: {PLATFORM}", "PLATFORM",
              platformType);

    if (platformType == "nvl72")
    {
        return std::make_unique<NVL72PowerControl>(io, conn, configPath, node,
                                                   appState);
    }
    else if (platformType == "c2")
    {
        return std::make_unique<C2PowerControl>(io, conn, configPath, node,
                                                appState);
    }
    else if (platformType == "vr-nvl8")
    {
        return std::make_unique<VRNVL8PowerControl>(io, conn, configPath, node,
                                                    appState);
    }

    lg2::error("Unknown platform-type '{PLATFORM}', defaulting to nvl72",
               "PLATFORM", platformType);
    return std::make_unique<NVL72PowerControl>(io, conn, configPath, node,
                                               appState);
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

    std::shared_ptr<sdbusplus::asio::connection> conn =
        std::make_shared<sdbusplus::asio::connection>(io);

    const std::string configPath =
        "/usr/share/x86-power-control/power-config-host0.json";
    std::string platformType = readPlatformType(configPath);
    auto powerControl =
        createPowerControl(platformType, io, conn, configPath, node, appState);

    PowerRestoreController powerRestore(io, conn, node, *powerControl,
                                        appState);

#ifdef USE_PLT_RST
    sdbusplus::bus::match_t pltRstMatch(
        *conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='xyz.openbmc_project.State.Host.Misc'",
        [&powerControl](sdbusplus::message_t& msg) {
            powerControl->hostMiscHandler(msg);
        });
#endif

    // Always run power restore policy regardless of initial state
    // The policy controller will determine the appropriate action based on
    // the configured policy and saved power state
    powerRestore.run();

    // NMI source property monitor is initialized by the powerControl object
    // if NMIOut is configured in powerSignalMap
    powerControl->nmiSourcePropertyMonitor();

    lg2::info("Initializing power state.");

    // D-Bus interfaces (host, chassis, boot progress, buttons, OS state,
    // restart cause) are now initialized by the PowerControl base class
    // constructor

    powerControl->currentHostStateMonitor();

    powerControl->requestBusNames();

    io.run();

    return 0;
}
