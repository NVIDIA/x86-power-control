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
    addRequiredResource("BaseboardIOX", ResourceType::IOXPath);

    // VRPowerControl already registers the common Board 0 VR signals:
    //   Board0RunPowerEnable, Board0RunPowerPG, Board0PreSystemReset,
    //   Board0CpuShutdownForce, Board0CpuShutdownRequest,
    //   Board0CpuShutdownOk, CpuResetIndicator.
    //
    // Add StbyPwrOk to monitor the standby power domain that feeds the
    // IOX hosting the power-sequencing GPIOs.
    addRequiredSignal("StbyPwrOk", 0, GPIODirection::IN,
                      [this](bool state) { this->stbyPwrOkHandler(state); });

    // HostReadyPowerEnable is VC-256-specific, registered here.
    addRequiredSignal("HostReadyPowerEnable", 0, GPIODirection::OUT);

    // PDBMainPowerOk/PDBMainPowerEnable are the HSC (Hot Swap Controller)
    // equivalent of NVL72's PDB Main Power signals: P54V_HSC_PG-I / -EN-O.
    addRequiredSignal("PDBMainPowerOk", 0, GPIODirection::IN,
                      [this](bool state) {
                          this->pdbMainPowerOkHandler(state);
                      });
    addRequiredSignal("PDBMainPowerEnable", 0, GPIODirection::OUT);

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

    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available");
        return false;
    }

    return (
        board0RunPowerPG->gpioLine.get_value() == !board0RunPowerPG->polarity &&
        pdbMainPowerOk->gpioLine.get_value() == !pdbMainPowerOk->polarity);
}

void VC256PowerControl::handlePowerOnRequest()
{
    lg2::info(
        "Power On Request received. Setting GPIOs to default state for host state Off and Commencing Host Main Power On sequence.");

    auto board0RunPowerPG = getSignal("Board0RunPowerPG");
    if (!board0RunPowerPG || !board0RunPowerPG->gpioLine)
    {
        lg2::error(
            "CRITICAL: Board0RunPowerPG not available - cannot power on");
        return;
    }

    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available - cannot power on");
        return;
    }

    auto pdbMainPowerEnable = getSignal("PDBMainPowerEnable");
    if (!pdbMainPowerEnable || !pdbMainPowerEnable->gpioLine)
    {
        lg2::error(
            "CRITICAL: PDBMainPowerEnable not available - cannot power on");
        return;
    }

    if (board0RunPowerPG->gpioLine.get_value() == board0RunPowerPG->polarity &&
        pdbMainPowerOk->gpioLine.get_value() == pdbMainPowerOk->polarity)
    {
        lg2::info(
            "PDB Main Power and HPM Run Power is already enabled. Setting GPIOs for host state ON and transitioning to PowerState::On");
        setGPIOsForHostStateOn();
        action = PowerAction::NONE;
        setPowerState(PowerState::on);
        return;
    }

    setGPIOsForHostStateOff();
    lg2::info(
        "Asserting PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOk");
    action = PowerAction::POWER_ON;
    setGPIOOutput(pdbMainPowerEnable, pdbMainPowerEnable->polarity);
    startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
               Event::pdbMainPowerOkWatchdogTimerExpired);
    setPowerState(PowerState::waitForPDBMainPowerOk);
}

void VC256PowerControl::initiatePDBPowerOff()
{
    cancelTimer("HPM Power Good Watchdog Timer", hpmPowerGoodWatchdogTimer);

    auto pdbMainPowerOk = getSignal("PDBMainPowerOk");
    if (!pdbMainPowerOk || !pdbMainPowerOk->gpioLine)
    {
        lg2::error("CRITICAL: PDBMainPowerOk not available");
        // Fallback: assume worst case and transition to waitForPDBMainPowerOff
        lg2::info(
            "HPM Board 0 Run Power Good de-asserted. De-asserting PDB Main Power Enable. "
            "Starting PDB Main Power OK Watchdog Timer. Transitioning to PowerState::waitForPDBMainPowerOff.");
        setPowerState(PowerState::waitForPDBMainPowerOff);
        deassertPDBMainPower();
        startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
        return;
    }

    bool pdbMainPowerOkAsserted =
        pdbMainPowerOk->gpioLine.get_value() == pdbMainPowerOk->polarity;

    if (pdbMainPowerOkAsserted)
    {
        lg2::info(
            "HPM Board 0 Run Power Good de-asserted. PDBMainPowerOk is currently asserted. "
            "De-asserting PDB Main Power Enable. Starting PDB Main Power OK Watchdog Timer. "
            "Transitioning to PowerState::waitForPDBMainPowerOff to wait for de-assertion.");
        setPowerState(PowerState::waitForPDBMainPowerOff);
        deassertPDBMainPower();
        startTimer("PdbMainPowerOkWatchdogMs", pdbMainPowerOkWatchdogTimer,
                   Event::pdbMainPowerOkWatchdogTimerExpired);
    }
    else
    {
        // PDB Main Power OK is already de-asserted — bypass wait state
        lg2::info(
            "HPM Board 0 Run Power Good de-asserted. PDBMainPowerOk is already de-asserted. "
            "De-asserting PDB Main Power Enable. Bypassing PowerState::waitForPDBMainPowerOff...");
        deassertPDBMainPower();
        completeShutdownAndTransitionToOff(true);
    }
}

// ============================================================================
// Platform peripheral hooks
// ============================================================================

void VC256PowerControl::assertPlatformPeripherals()
{
    auto hostReadyPowerEnable = getSignal("HostReadyPowerEnable");
    if (!hostReadyPowerEnable)
    {
        return;
    }
    setGPIOOutput(hostReadyPowerEnable, hostReadyPowerEnable->polarity);
}

void VC256PowerControl::deassertPlatformPeripherals()
{
    auto hostReadyPowerEnable = getSignal("HostReadyPowerEnable");
    if (!hostReadyPowerEnable)
    {
        return;
    }
    setGPIOOutput(hostReadyPowerEnable, !hostReadyPowerEnable->polarity);
}

void VC256PowerControl::setDefaultValues()
{
    lg2::info(
        "Defining VC-256 platform GPIOs asserted and de-asserted states based on host state ON and OFF");

    auto hostReadyPowerEnable = getSignal("HostReadyPowerEnable");
    if (!hostReadyPowerEnable)
    {
        return;
    }

    // Host Ready Power Enable: ON=Asserted, OFF=DeAsserted
    hostReadyPowerEnable->defaultStateHostStateOn = DefaultState::Asserted;
    hostReadyPowerEnable->defaultStateHostStateOff = DefaultState::DeAsserted;

    // Call parent to set common VR/HPM defaults
    VRPowerControl::setDefaultValues();

    lg2::info(
        "VC-256 GPIOs asserted and de-asserted states defined successfully");
}

// ============================================================================
// PDB Main Power (HSC) overrides
// ============================================================================

void VC256PowerControl::pdbMainPowerOkHandler(bool state)
{
    lg2::info("PDBMainPowerOk GPIO event: value={VALUE}", "VALUE",
              static_cast<int>(state));

    auto configPtr = getSignal("PDBMainPowerOk");
    if (!configPtr)
    {
        return;
    }

    Event powerControlEvent = (state == configPtr->polarity)
                                  ? Event::pdbMainPowerOkAssert
                                  : Event::pdbMainPowerOkDeAssert;

    if (checkAndHandlePdbMainPowerOkFault(powerControlEvent))
    {
        return; // Fault was handled, exit early
    }

    this->sendPowerControlEvent(powerControlEvent);
}

bool VC256PowerControl::checkAndHandlePdbMainPowerOkFault(
    Event powerControlEvent)
{
    if (powerControlEvent == Event::pdbMainPowerOkDeAssert)
    {
        if (powerState != PowerState::waitForPDBMainPowerOff)
        {
            lg2::error(
                "POWER FAULT DETECTED: PDBMainPowerOk de-asserted unexpectedly while in power state {STATE}. "
                "Setting GPIO states to match Host State OFF. Transitioning to Host State OFF.",
                "STATE", getPowerStateName());

            action = PowerAction::NONE;
            logResourceEvent(
                "ResourceErrorsDetected",
                {"Host0",
                 "PDB Main Power OK de-asserted unexpectedly while in power state {STATE}.",
                 "STATE", getPowerStateName()},
                "xyz.openbmc_project.Logging.Entry.Level.Error");
            transitionToOffStateWithRunPowerCheck();

            return true;
        }
        // else: Expected de-assertion in waitForPDBMainPowerOff state
    }

    return false;
}

void VC256PowerControl::deassertPDBMainPower()
{
    auto pdbMainPowerEnable = getSignal("PDBMainPowerEnable");
    if (!pdbMainPowerEnable)
    {
        return;
    }

    setGPIOOutput(pdbMainPowerEnable, !pdbMainPowerEnable->polarity);
}

void VC256PowerControl::validateTimerConfigs()
{
    for (const auto& timerName : vc256RequiredTimeoutValues)
    {
        if (TimerMap.find(timerName) == TimerMap.end())
        {
            lg2::error(
                "Required VC-256 timer config '{TIMER}' not found in config",
                "TIMER", timerName);
            throw std::runtime_error(
                "VC256PowerControl: Required timer config missing: " +
                timerName);
        }
    }

    VRPowerControl::validateTimerConfigs();

    lg2::info(
        "VC-256 timer configuration validation complete - all required timers present");
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

} // namespace power_control
