// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright OpenBMC Authors

#include "cmx_power_control.hpp"

#include <phosphor-logging/lg2.hpp>

namespace power_control
{
using Event = PowerControl::Event;

CMXPowerControl::CMXPowerControl(
    boost::asio::io_context& ioContext,
    std::shared_ptr<sdbusplus::asio::connection> conn,
    const std::string& configFilePath, const std::string& node,
    PersistentState& appState) :
    VRPowerControl(ioContext, conn, configFilePath, node, appState)
{
    // HPM IOX hosting the sequencing GPIOs, and PDB IOX hosting the HPM
    // standby power good.
    addRequiredResource("Board0IoxPath", ResourceType::IOXPath);
    addRequiredResource("PdbIoxPath", ResourceType::IOXPath);

    // VRPowerControl already registers the common Board 0 VR signals:
    //   Board0RunPowerEnable, Board0RunPowerPG, Board0PreSystemReset,
    //   Board0CpuShutdownForce, Board0CpuShutdownRequest,
    //   Board0CpuShutdownOk, CpuResetIndicator.

    // CPLD_READY gates run power sequencing in the CPLD.
    addRequiredSignal("CpldReady", 0, GPIODirection::IN,
                      [this](bool state) { this->cpldReadyHandler(state); });

    // HPM standby power good witnesses the domain feeding the sequencing IOX.
    addRequiredSignal("StbyPwrOk", 0, GPIODirection::IN,
                      [this](bool state) { this->stbyPwrOkHandler(state); });

    PowerControl::validateRequiredResources();
    PowerControl::validateRequiredSignals();
    validateTimerConfigs();
    setDefaultValues();

    // CMX is ON iff Board0RunPowerPG is asserted.
    initializePowerStateFromHardware(powerIndicators, true);
    registerCpuBootDoneSetterMethod();
    initializeHostStateInterface();
}

// ============================================================================
// Pure-virtual overrides
// ============================================================================

bool CMXPowerControl::isSystemPowerOff()
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

void CMXPowerControl::handlePowerOnRequest()
{
    lg2::info(
        "Power On Request received, commencing HPM-only Power On sequence");

    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - cannot power on");
        return;
    }

    if (board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity)
    {
        lg2::info("Board 0 Run Power Good is already asserted. Setting GPIOs "
                  "for host state ON and transitioning to PowerState::on.");
        setGPIOsForHostStateOn();
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
    }
    else
    {
        setGPIOsForHostStateOff();
        lg2::info("Commencing HPM Board Power Sequencing. Transitioning to "
                  "PowerState::waitForHPMPowerGoodAssert.");
        action = PowerAction::POWER_ON;
        transitionToHPMPowerGoodAssertState();
    }
}

void CMXPowerControl::initiatePDBPowerOff()
{
    // No PDB stage to tear down; the CPLD removes the rails.
    lg2::info(
        "HPM Board 0 Run Power Good de-asserted, dispatching shutdown action");
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);
    applyShutdownAction();
}

// ============================================================================
// Power-on guards
// ============================================================================

bool CMXPowerControl::canAcceptPowerOnRequest(std::string& reason)
{
    if (stbyPowerLost)
    {
        reason =
            "System in degraded state due to prior standby power loss - AC cycle required to recover power sequencing";
        return false;
    }

    auto cpldReady = getSignal("CpldReady");
    if (!cpldReady || !cpldReady->gpioLine)
    {
        reason = "CpldReady not available - cannot power on";
        return false;
    }

    if (cpldReady->gpioLine.get_value() != cpldReady->polarity)
    {
        reason = "CPLD is not ready - power on rejected";
        return false;
    }

    return true;
}

bool CMXPowerControl::shouldIgnoreEvent(const std::string& signalName)
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
    // de-assert event. Read the witness directly to set the flag eagerly.
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

void CMXPowerControl::cpldReadyHandler(bool state)
{
    auto configPtr = getSignal("CpldReady");
    if (!configPtr)
    {
        return;
    }

    if (state == configPtr->polarity)
    {
        lg2::info("CPLD ready asserted");
    }
    else
    {
        lg2::warning("CPLD ready de-asserted in power state {STATE}", "STATE",
                     getPowerStateName());
    }
}

void CMXPowerControl::stbyPwrOkHandler(bool state)
{
    auto configPtr = getSignal("StbyPwrOk");
    if (!configPtr)
    {
        lg2::error("CRITICAL: StbyPwrOk signal not available");
        return;
    }

    if (state != configPtr->polarity)
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

void CMXPowerControl::markStandbyLost()
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
// Defensive overrides - PDB states should never be entered on CMX
// ============================================================================

std::function<void(Event)> CMXPowerControl::getPowerStateHandler()
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error("Invalid state: powerState is a PDB wait state, but CMX "
                       "has no PDB hardware - dropping event");
            return {};

        default:
            return VRPowerControl::getPowerStateHandler();
    }
}

std::string_view CMXPowerControl::getHostState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error("Invalid state: powerState is a PDB wait state in "
                       "getHostState, returning Off");
            return "xyz.openbmc_project.State.Host.HostState.Off";

        default:
            break;
    }
    return VRPowerControl::getHostState();
}

std::string_view CMXPowerControl::getChassisState() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            lg2::error("Invalid state: powerState is a PDB wait state in "
                       "getChassisState, returning Off");
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";

        default:
            break;
    }
    return VRPowerControl::getChassisState();
}

std::string CMXPowerControl::getPowerStateName() const
{
    switch (powerState)
    {
        case PowerState::waitForPDBMainPowerOk:
        case PowerState::waitForPDBMainPowerOff:
            return "Invalid state: powerState is a PDB wait state";

        default:
            break;
    }
    return VRPowerControl::getPowerStateName();
}

} // namespace power_control
