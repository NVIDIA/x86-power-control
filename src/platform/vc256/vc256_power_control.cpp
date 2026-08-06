// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "vc256_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

#include <string>

namespace power_control
{
using Event = PowerControl::Event;

VC256PowerControl::VC256PowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node, appState)
{
    addRequiredResource("Board0IoxPath", ResourceType::IOXPath);

    // VRPowerControl already registers the common Board 0 VR signals:
    //   Board0RunPowerEnable, Board0RunPowerPG, Board0PreSystemReset,
    //   Board0CpuShutdownForce, Board0CpuShutdownRequest,
    //   Board0CpuShutdownOk, CpuResetIndicator.
    //
    // Add StbyPwrOk to monitor the standby power domain that feeds the
    // IOX hosting the power-sequencing GPIOs.
    addRequiredSignal("StbyPwrOk", 0, GPIODirection::IN,
                      [this](bool state) { this->stbyPwrOkHandler(state); });

    PowerControl::validateRequiredResources();
    PowerControl::validateRequiredSignals();
    validateTimerConfigs();
    setDefaultValues();

    initializePowerStateFromHardware(powerIndicators, true);
    registerCpuBootDoneSetterMethod();
    initializeHostStateInterface();
}

// ============================================================================
// Pure-virtual overrides
// ============================================================================

bool VC256PowerControl::isSystemPowerOff()
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

void VC256PowerControl::handlePowerOnRequest()
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
        "Commencing HPM run-power sequencing. Transitioning to PowerState::waitForHPMPowerGoodAssert.");
    action = PowerAction::POWER_ON;
    transitionToHPMPowerGoodAssertState();
}

void VC256PowerControl::initiatePDBPowerOff()
{
    lg2::info(
        "HPM Board 0 Run Power Good de-asserted. No PDB - dispatching shutdown action.");
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);
    applyShutdownAction();
}

// ============================================================================
// Standby power guard overrides
// ============================================================================

bool VC256PowerControl::canAcceptPowerOnRequest(std::string& reason)
{
    if (stbyPowerLost)
    {
        reason =
            "System in degraded state due to prior standby power loss - AC cycle required to recover power sequencing";
        return false;
    }
    return true;
}

bool VC256PowerControl::shouldIgnoreEvent(const std::string& signalName)
{
    // Always deliver StbyPwrOk so state changes on the witness are observed.
    if (signalName == "StbyPwrOk")
    {
        return false;
    }

    if (stbyPowerLost)
    {
        return true;
    }

    // A noise event from a dying IOX may arrive before the StbyPwrOk
    // de-assert event due to kernel per-chip delivery ordering. Read the
    // witness directly to set the flag eagerly before this event's handler
    // runs on a dead line.
    auto stby = getSignal("StbyPwrOk");
    if (stby && stby->gpioLine)
    {
        int value = 0;
        try
        {
            value = stby->gpioLine.get_value();
        }
        catch (const std::exception&)
        {
            return false;
        }

        if (value != stby->polarity)
        {
            markStandbyLost();
            return true;
        }
    }

    return false;
}

// ============================================================================
// Standby power private helpers
// ============================================================================

void VC256PowerControl::stbyPwrOkHandler(bool state)
{
    auto configPtr = getSignal("StbyPwrOk");
    if (!configPtr)
    {
        lg2::error("CRITICAL: StbyPwrOk signal not available");
        return;
    }

    const bool asserted = (state == configPtr->polarity);

    if (!asserted)
    {
        markStandbyLost();
        return;
    }

    // Re-assert. Recovery requires AC cycle or BMC reboot.
    lg2::info(
        "Standby power domain restored - AC cycle recommended to recover.");
    logResourceEvent(
        "ResourceEvent",
        {"Host0",
         "Standby power domain restored - AC cycle recommended to recover."},
        "xyz.openbmc_project.Logging.Entry.Level.Informational");
}

void VC256PowerControl::markStandbyLost()
{
    if (stbyPowerLost)
    {
        return;
    }
    stbyPowerLost = true;

    lg2::error(
        "Standby power domain lost - power sequencing hardware unavailable. AC cycle recommended to recover.");
    logResourceEvent(
        "ResourceErrorsDetected",
        {"Host0",
         "Standby power domain lost - power sequencing hardware unavailable. AC cycle recommended to recover."},
        "xyz.openbmc_project.Logging.Entry.Level.Error");

    setPowerState(PowerState::off);
}

// ============================================================================
// Defensive overrides - PDB states should never be entered on this platform
// ============================================================================

std::function<void(Event)> VC256PowerControl::getPowerStateHandler()
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error(
                "VC-256: Unexpected PDB power state - platform has no PDB. "
                "Events in this state will be dropped.");
            return {};

        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

std::string_view VC256PowerControl::getHostState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error(
                "VC-256: Unexpected PDB power state in getHostState, returning Off");
            return "xyz.openbmc_project.State.Host.HostState.Off";

        default:
            break;
    }
    return VRPowerControl::getHostState();
}

std::string_view VC256PowerControl::getChassisState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error(
                "VC-256: Unexpected PDB power state in getChassisState, returning Off");
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";

        default:
            break;
    }
    return VRPowerControl::getChassisState();
}

std::string VC256PowerControl::getPowerStateName() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            return "Unexpected PDB state (platform has no PDB)";

        default:
            break;
    }
    return VRPowerControl::getPowerStateName();
}

} // namespace power_control
