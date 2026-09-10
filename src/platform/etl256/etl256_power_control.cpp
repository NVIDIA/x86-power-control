// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "etl256_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <string>

namespace power_control
{
// Type aliases for convenience
using Event = PowerControl::Event;
ETL256PowerControl::ETL256PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node, appState)
{
    // Required resources (IOX paths). ETL256 power-control GPIOs span the
    // HPM/P4508 Board 0 IO expanders and the SMM USB-power IO expander.
    // The deployed JSON must declare these sysfs paths so startup fails early
    // if a required expander is not present.
    addRequiredResource("Board0IoxPath", ResourceType::IOXPath);
    addRequiredResource("Board0ResetIoxPath", ResourceType::IOXPath);
    addRequiredResource("SmmUsbPowerIoxPath", ResourceType::IOXPath);

    // VRPowerControl already registers the common Board 0 VR signals:
    //   Board0RunPowerEnable, Board0RunPowerPG, Board0PreSystemReset,
    //   Board0CpuShutdownForce, Board0CpuShutdownRequest,
    //   Board0CpuShutdownOk, CpuResetIndicator.
    //
    // ETL256 adds USBPowerEnable, but has no PDB signals, no 12V PDB rails,
    // no E1S/SSD platform peripherals, and no Board 1.

    addRequiredSignal("USBPowerEnable", 0, GPIODirection::OUT);

    PowerControl::validateRequiredResources();
    PowerControl::validateRequiredSignals();
    validateTimerConfigs();
    setDefaultValues();

    // ETL256 is ON iff the HPM module power-good signal is asserted.
    initializePowerStateFromHardware(powerIndicators, true);

    // Publish the host/chassis/GPIO D-Bus interfaces after initial state is
    // derived from hardware, so clients see the correct state immediately.
    initializeHostStateInterface();
}

// ============================================================================
// Pure-virtual overrides
// ============================================================================

bool ETL256PowerControl::isSystemPowerOff()
{
    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error("CRITICAL: Board0RunPowerPG not available");
        return false;
    }

    return board0RunPowerPG->gpioLine.get_value() ==
           !board0RunPowerPG->polarity;
}

void ETL256PowerControl::handlePowerOnRequest()
{
    lg2::info("Power On Request received. Commencing HPM run-power sequence.");

    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - cannot power on");
        return;
    }

    if (board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity)
    {
        lg2::info(
            "Board 0 Run Power Good is already asserted. Setting GPIOs for host state ON and transitioning to PowerState::on.");
        setGPIOsForHostStateOn();
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
        return;
    }

    setGPIOsForHostStateOff();

    lg2::info(
        "Commencing ETL256 HPM run-power sequencing. Transitioning to PowerState::waitForHPMPowerGoodAssert.");
    action = PowerAction::POWER_ON;
    transitionToHPMPowerGoodAssertState();
}

bool ETL256PowerControl::canAcceptPowerOnRequest(std::string& reason)
{
    constexpr const char* standbyPowerGoodPath =
        "/run/bmc-state/HPM_B0_M0_STANDBY_POWER_PG";

    std::ifstream standbyPowerGoodFile(standbyPowerGoodPath);
    if (!standbyPowerGoodFile.is_open())
    {
        reason =
            "ETL256 standby power-good state file is missing - cannot safely start run-power sequencing";
        return false;
    }

    std::string value;
    std::getline(standbyPowerGoodFile, value);

    value.erase(std::remove_if(value.begin(), value.end(),
                               [](unsigned char c) { return std::isspace(c); }),
                value.end());

    if (value != "1")
    {
        reason =
            "ETL256 standby power is not good - cannot safely start run-power sequencing";
        return false;
    }

    return true;
}

// ============================================================================
// Virtual overrides - extend VR defaults
// ============================================================================

void ETL256PowerControl::assertPlatformPeripherals()
{
    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        lg2::error("CRITICAL: USBPowerEnable not available");
        return;
    }

    setGPIOOutput(usbPowerEnable, usbPowerEnable->polarity);
}

void ETL256PowerControl::deassertPlatformPeripherals()
{
    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        lg2::error("CRITICAL: USBPowerEnable not available");
        return;
    }

    setGPIOOutput(usbPowerEnable, !usbPowerEnable->polarity);
}

void ETL256PowerControl::initiatePDBPowerOff()
{
    // ETL256 has no PDB to tear down. Dispatch the shutdown action immediately.
    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. ETL256 has no PDB - dispatching shutdown action.");
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);
    applyShutdownAction();
}

void ETL256PowerControl::setDefaultValues()
{
    lg2::info(
        "Defining ETL256 GPIO asserted and de-asserted states based on host state ON and OFF");

    auto usbPowerEnable = getSignal("USBPowerEnable");
    if (!usbPowerEnable)
    {
        lg2::error("CRITICAL: USBPowerEnable not available");
    }
    else
    {
        // USB Power Enable: ON=Asserted, OFF=DeAsserted
        usbPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
        usbPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;
    }

    VRPowerControl::setDefaultValues();

    auto board0PreSystemReset = getSignal("Board0PreSystemReset");
    if (!board0PreSystemReset)
    {
        lg2::error("CRITICAL: Board0PreSystemReset not available");
    }
    else
    {
        // ETL256 shutdown cleanup de-asserts PRE_SYS_RST_L after run power
        // drops, so the steady host-off default should be de-asserted.
        board0PreSystemReset->defaultStateHostStateOff =
            DefaultState::DeAsserted;
    }

    lg2::info("ETL256 GPIO states defined successfully");
}

// ============================================================================
// Defensive overrides - PDB states should never be entered on ETL256
// ============================================================================

std::function<void(Event)> ETL256PowerControl::getPowerStateHandler()
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error(
                "ETL256: Unexpected PDB power state - ETL256 has no PDB. "
                "Events in this state will be dropped.");
            return {};

        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

std::string_view ETL256PowerControl::getHostState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error("ETL256: Unexpected PDB power state in getHostState, "
                       "returning Off");
            return "xyz.openbmc_project.State.Host.HostState.Off";

        default:
            break;
    }
    return VRPowerControl::getHostState();
}

std::string_view ETL256PowerControl::getChassisState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error("ETL256: Unexpected PDB power state in getChassisState, "
                       "returning Off");
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";

        default:
            break;
    }
    return VRPowerControl::getChassisState();
}

std::string ETL256PowerControl::getPowerStateName() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            return "Unexpected PDB state (ETL256 has no PDB)";

        default:
            break;
    }
    return VRPowerControl::getPowerStateName();
}

} // namespace power_control
